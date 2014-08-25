#include "stkscenemanager.hpp"
#include "stkmesh.hpp"
#include "stkinstancedscenenode.hpp"
#include "irr_driver.hpp"
#include <ISceneManager.h>
#include <ISceneNode.h>
#include "stkanimatedmesh.hpp"
#include "stkmeshscenenode.hpp"
#include "utils/ptr_vector.hpp"
#include <ICameraSceneNode.h>
#include <SViewFrustum.h>
#include "callbacks.hpp"
#include "utils/cpp2011.hpp"
#include <omp.h>
#include "modes/world.hpp"
#include "tracks/track.hpp"

template <typename...Args>
static void
FillInstances_impl(std::vector<std::pair<GLMesh *, scene::ISceneNode *> > InstanceList, InstanceData *IB, DrawElementsIndirectCommand *CommandBuffer,
    size_t &InstanceBufferOffset, size_t &CommandBufferOffset)
{
    // Should never be empty
    GLMesh *mesh = InstanceList.front().first;

    InstanceData * InstanceBuffer;

    if (UserConfigParams::m_azdo)
    {
        DrawElementsIndirectCommand &CurrentCommand = CommandBuffer[CommandBufferOffset];
        CurrentCommand.baseVertex = mesh->vaoBaseVertex;
        CurrentCommand.count = mesh->IndexCount;
        CurrentCommand.firstIndex = mesh->vaoOffset / 2;
        CurrentCommand.baseInstance = InstanceBufferOffset;
        CurrentCommand.instanceCount = InstanceList.size();
        InstanceBuffer = IB;
    }
    else if (irr_driver->hasARB_base_instance())
    {
        mesh->vaoBaseInstance = InstanceBufferOffset;
        InstanceBuffer = IB;
    }
    else
    {
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, mesh->instance_buffer);
        InstanceBuffer = (InstanceData *)glMapBufferRange(GL_ARRAY_BUFFER, 0, 1000 * sizeof(InstanceData), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
        InstanceBufferOffset = 0;
    }

    CommandBufferOffset++;

    irr_driver->IncreasePolyCount(InstanceList.size() * mesh->IndexCount / 3);

    for (unsigned i = 0; i < InstanceList.size(); i++)
    {
        auto &Tp = InstanceList[i];
        InstanceData &Instance = InstanceBuffer[InstanceBufferOffset++];
        scene::ISceneNode *node = Tp.second;
        const core::matrix4 &mat = node->getAbsoluteTransformation();
        const core::vector3df &Origin = mat.getTranslation();
        const core::vector3df &Orientation = mat.getRotationDegrees();
        const core::vector3df &Scale = mat.getScale();
        Instance.Origin.X = Origin.X;
        Instance.Origin.Y = Origin.Y;
        Instance.Origin.Z = Origin.Z;
        Instance.Orientation.X = Orientation.X;
        Instance.Orientation.Y = Orientation.Y;
        Instance.Orientation.Z = Orientation.Z;
        Instance.Scale.X = Scale.X;
        Instance.Scale.Y = Scale.Y;
        Instance.Scale.Z = Scale.Z;
        Instance.Texture = mesh->TextureHandles[0];
        Instance.SecondTexture = mesh->TextureHandles[1];
    }

    if (!irr_driver->hasARB_base_instance())
        glUnmapBuffer(GL_ARRAY_BUFFER);
}


static
void FillInstances(const std::map<scene::IMeshBuffer *, std::vector<std::pair<GLMesh *, scene::ISceneNode*> > > &GatheredGLMesh, std::vector<STK::Tuple<GLMesh *, size_t> > *InstancedList,
    InstanceData *InstanceBuffer, DrawElementsIndirectCommand *CommandBuffer, size_t &InstanceBufferOffset, size_t &CommandBufferOffset)
{
    auto It = GatheredGLMesh.begin(), E = GatheredGLMesh.end();
    for (; It != E; ++It)
    {
        FillInstances_impl(It->second, InstanceBuffer, CommandBuffer, InstanceBufferOffset, CommandBufferOffset);
        if (!UserConfigParams::m_azdo)
            InstancedList->push_back(STK::make_tuple(It->second.front().first, It->second.size()));
    }
}

static
void FillInstancesGrass(const std::map<scene::IMeshBuffer *, std::vector<std::pair<GLMesh *, scene::ISceneNode*> > > &GatheredGLMesh, std::vector<STK::Tuple<GLMesh *, size_t, core::vector3df, core::vector3df> > *InstancedList,
    InstanceData *InstanceBuffer, DrawElementsIndirectCommand *CommandBuffer, size_t &InstanceBufferOffset, size_t &CommandBufferOffset)
{
    auto It = GatheredGLMesh.begin(), E = GatheredGLMesh.end();
    core::vector3df dir = core::vector3df(0., 0., 0.);
    SunLightProvider * const cb = (SunLightProvider *)irr_driver->getCallback(ES_SUNLIGHT);
    for (; It != E; ++It)
    {
        FillInstances_impl(It->second, InstanceBuffer, CommandBuffer, InstanceBufferOffset, CommandBufferOffset);
        if (!UserConfigParams::m_azdo)
            InstancedList->push_back(STK::make_tuple(It->second.front().first, It->second.size(), dir, cb->getPosition()));
    }
}

static std::map <scene::IMeshBuffer *, std::vector<std::pair<GLMesh *, scene::ISceneNode*> > > MeshForSolidPass[MAT_COUNT];
static std::map <scene::IMeshBuffer *, std::vector<std::pair<GLMesh *, scene::ISceneNode*> > > MeshForShadowPass[4][MAT_COUNT];


static void
parseSceneManager(core::list<scene::ISceneNode*> List, std::vector<STKMeshSceneNode *> &ImmediateDraw,
    scene::ICameraSceneNode *shadowCams[4],
    bool IsCulledForSolid, bool IsCulledForShadow[4])
{
    core::list<scene::ISceneNode*>::Iterator I = List.begin(), E = List.end();
    for (; I != E; ++I)
    {
        if (!(*I)->isVisible())
            continue;
        (*I)->updateAbsolutePosition();
        core::aabbox3d<f32> tbox = (*I)->getBoundingBox();
        (*I)->getAbsoluteTransformation().transformBoxEx(tbox);


        const scene::ICameraSceneNode* cam = irr_driver->getSceneManager()->getActiveCamera();
        IsCulledForSolid = !(tbox.intersectsWithBox(cam->getViewFrustum()->getBoundingBox()));

        for (unsigned i = 0; i < 4; ++i)
        {
            const scene::ICameraSceneNode* cam = shadowCams[i];
            IsCulledForShadow[i] = !(tbox.intersectsWithBox(cam->getViewFrustum()->getBoundingBox()));
        }

        if (IsCulledForSolid && IsCulledForShadow[0] && IsCulledForShadow[1] && IsCulledForShadow[2] && IsCulledForShadow[3])
            continue;

        if (STKAnimatedMesh *node = dynamic_cast<STKAnimatedMesh*>(*I))
        {
            node->update();
            for (unsigned Mat = 0; Mat < MAT_COUNT; ++Mat)
            {
                GLMesh *mesh;
                if (!IsCulledForSolid)
                {
                    for_in(mesh, node->MeshSolidMaterial[Mat])
                        MeshForSolidPass[Mat][mesh->mb].push_back(std::make_pair(mesh, node));
                }
                for (unsigned cascade = 0; cascade < 4; ++cascade)
                {
                    if (!IsCulledForShadow[cascade])
                    {
                        for_in(mesh, node->MeshSolidMaterial[Mat])
                            MeshForShadowPass[cascade][Mat][mesh->mb].push_back(std::make_pair(mesh, node));
                    }
                }
            }
            // Transparent
            if (!IsCulledForSolid)
            {
                GLMesh *mesh;
                if (World::getWorld() && World::getWorld()->isFogEnabled())
                {
                    const Track * const track = World::getWorld()->getTrack();

                    // Todo : put everything in a ubo
                    const float fogmax = track->getFogMax();
                    const float startH = track->getFogStartHeight();
                    const float endH = track->getFogEndHeight();
                    const float start = track->getFogStart();
                    const float end = track->getFogEnd();
                    const video::SColor tmpcol = track->getFogColor();

                    video::SColorf col(tmpcol.getRed() / 255.0f,
                        tmpcol.getGreen() / 255.0f,
                        tmpcol.getBlue() / 255.0f);

                    for_in(mesh, node->TransparentMesh[TM_DEFAULT])
                        pushVector(ListBlendTransparentFog::getInstance(), mesh, node->getAbsoluteTransformation(), mesh->TextureMatrix,
                        fogmax, startH, endH, start, end, col);
                    for_in(mesh, node->TransparentMesh[TM_ADDITIVE])
                        pushVector(ListAdditiveTransparentFog::getInstance(), mesh, node->getAbsoluteTransformation(), mesh->TextureMatrix,
                        fogmax, startH, endH, start, end, col);
                }
                else
                {
                    for_in(mesh, node->TransparentMesh[TM_DEFAULT])
                        pushVector(ListBlendTransparent::getInstance(), mesh, node->getAbsoluteTransformation(), mesh->TextureMatrix);
                    for_in(mesh, node->TransparentMesh[TM_ADDITIVE])
                        pushVector(ListAdditiveTransparent::getInstance(), mesh, node->getAbsoluteTransformation(), mesh->TextureMatrix);
                }
            }
        }
        if (STKMeshSceneNode *node = dynamic_cast<STKMeshSceneNode*>(*I))
        {
            node->update();
            if (node->isImmediateDraw())
            {
                ImmediateDraw.push_back(node);
                continue;
            }

            for (unsigned Mat = 0; Mat < MAT_COUNT; ++Mat)
            {
                GLMesh *mesh;
                if (!IsCulledForSolid)
                {
                    for_in(mesh, node->MeshSolidMaterials[Mat])
                        MeshForSolidPass[Mat][mesh->mb].push_back(std::make_pair(mesh, node));
                }
                for (unsigned cascade = 0; cascade < 4; ++cascade)
                {
                    if (!IsCulledForShadow[cascade])
                    {
                        for_in(mesh, node->MeshSolidMaterials[Mat])
                            MeshForShadowPass[cascade][Mat][mesh->mb].push_back(std::make_pair(mesh, node));
                    }
                }
            }
            // Transparent
            if (!IsCulledForSolid)
            {
                GLMesh *mesh;

                if (World::getWorld() && World::getWorld()->isFogEnabled())
                {
                    const Track * const track = World::getWorld()->getTrack();

                    // Todo : put everything in a ubo
                    const float fogmax = track->getFogMax();
                    const float startH = track->getFogStartHeight();
                    const float endH = track->getFogEndHeight();
                    const float start = track->getFogStart();
                    const float end = track->getFogEnd();
                    const video::SColor tmpcol = track->getFogColor();

                    video::SColorf col(tmpcol.getRed() / 255.0f,
                        tmpcol.getGreen() / 255.0f,
                        tmpcol.getBlue() / 255.0f);

                    for_in(mesh, node->TransparentMesh[TM_DEFAULT])
                        pushVector(ListBlendTransparentFog::getInstance(), mesh, node->getAbsoluteTransformation(), mesh->TextureMatrix,
                            fogmax, startH, endH, start, end, col);
                    for_in(mesh, node->TransparentMesh[TM_ADDITIVE])
                        pushVector(ListAdditiveTransparentFog::getInstance(), mesh, node->getAbsoluteTransformation(), mesh->TextureMatrix,
                            fogmax, startH, endH, start, end, col);
                }
                else
                {
                    for_in(mesh, node->TransparentMesh[TM_DEFAULT])
                        pushVector(ListBlendTransparent::getInstance(), mesh, node->getAbsoluteTransformation(), mesh->TextureMatrix);
                    for_in(mesh, node->TransparentMesh[TM_ADDITIVE])
                        pushVector(ListAdditiveTransparent::getInstance(), mesh, node->getAbsoluteTransformation(), mesh->TextureMatrix);
                }
                for_in(mesh, node->TransparentMesh[TM_DISPLACEMENT])
                    pushVector(ListDisplacement::getInstance(), mesh, node->getAbsoluteTransformation());
            }
        }

        parseSceneManager((*I)->getChildren(), ImmediateDraw, shadowCams, IsCulledForSolid, IsCulledForShadow);
    }
}

void IrrDriver::PrepareDrawCalls()
{
    ListBlendTransparent::getInstance()->clear();
    ListAdditiveTransparent::getInstance()->clear();
    ListBlendTransparentFog::getInstance()->clear();
    ListAdditiveTransparentFog::getInstance()->clear();
    ListDisplacement::getInstance()->clear();
    for (unsigned Mat = 0; Mat < MAT_COUNT; ++Mat)
    {
        MeshForSolidPass[Mat].clear();
        for (unsigned cascade = 0; cascade < 4; ++cascade)
            MeshForShadowPass[cascade][Mat].clear();
    }
    core::list<scene::ISceneNode*> List = m_scene_manager->getRootSceneNode()->getChildren();
    std::vector<STKMeshSceneNode *> ImmediateDrawList;
    bool isCulled[4] = {};
    parseSceneManager(List, ImmediateDrawList, m_shadow_camnodes, false, isCulled);
    InstanceData *InstanceBuffer = VAOManager::getInstance()->getInstanceBuffer(InstanceTypeDefault);
    DrawElementsIndirectCommand * CmdBuffer = SolidPassCmd::getInstance()->Ptr;
    InstanceData *ShadowInstanceBuffer = VAOManager::getInstance()->getInstanceBuffer(InstanceTypeShadow);
    DrawElementsIndirectCommand *ShadowCmdBuffer = ShadowPassCmd::getInstance()->Ptr;
#pragma omp parallel sections
    {
#pragma omp section
        {
            irr_driver->setPhase(SOLID_NORMAL_AND_DEPTH_PASS);
            if (!UserConfigParams::m_azdo)
            {
                ListInstancedMatDefault::getInstance()->clear();
                ListInstancedMatAlphaRef::getInstance()->clear();
                ListInstancedMatGrass::getInstance()->clear();
                ListInstancedMatNormalMap::getInstance()->clear();
                ListInstancedMatSphereMap::getInstance()->clear();
                ListInstancedMatDetails::getInstance()->clear();
                ListInstancedMatUnlit::getInstance()->clear();
            }

            size_t offset = 0, current_cmd = 0;

            // Default Material
            SolidPassCmd::getInstance()->Offset[MAT_DEFAULT] = current_cmd;
            FillInstances(MeshForSolidPass[MAT_DEFAULT], ListInstancedMatDefault::getInstance(), InstanceBuffer, CmdBuffer, offset, current_cmd);
            SolidPassCmd::getInstance()->Size[MAT_DEFAULT] = current_cmd - SolidPassCmd::getInstance()->Offset[MAT_DEFAULT];
            // Alpha Ref
            SolidPassCmd::getInstance()->Offset[MAT_ALPHA_REF] = current_cmd;
            FillInstances(MeshForSolidPass[MAT_ALPHA_REF], ListInstancedMatAlphaRef::getInstance(), InstanceBuffer, CmdBuffer, offset, current_cmd);
            SolidPassCmd::getInstance()->Size[MAT_ALPHA_REF] = current_cmd - SolidPassCmd::getInstance()->Offset[MAT_ALPHA_REF];
            // Unlit
            SolidPassCmd::getInstance()->Offset[MAT_UNLIT] = current_cmd;
            FillInstances(MeshForSolidPass[MAT_UNLIT], ListInstancedMatUnlit::getInstance(), InstanceBuffer, CmdBuffer, offset, current_cmd);
            SolidPassCmd::getInstance()->Size[MAT_UNLIT] = current_cmd - SolidPassCmd::getInstance()->Offset[MAT_UNLIT];
            // Spheremap
            SolidPassCmd::getInstance()->Offset[MAT_SPHEREMAP] = current_cmd;
            FillInstances(MeshForSolidPass[MAT_SPHEREMAP], ListInstancedMatSphereMap::getInstance(), InstanceBuffer, CmdBuffer, offset, current_cmd);
            SolidPassCmd::getInstance()->Size[MAT_SPHEREMAP] = current_cmd - SolidPassCmd::getInstance()->Offset[MAT_SPHEREMAP];
            // Detail
            SolidPassCmd::getInstance()->Offset[MAT_DETAIL] = current_cmd;
            FillInstances(MeshForSolidPass[MAT_DETAIL], ListInstancedMatDetails::getInstance(), InstanceBuffer, CmdBuffer, offset, current_cmd);
            SolidPassCmd::getInstance()->Size[MAT_DETAIL] = current_cmd - SolidPassCmd::getInstance()->Offset[MAT_DETAIL];
            // Normal Map
            SolidPassCmd::getInstance()->Offset[MAT_NORMAL_MAP] = current_cmd;
            FillInstances(MeshForSolidPass[MAT_NORMAL_MAP], ListInstancedMatNormalMap::getInstance(), InstanceBuffer, CmdBuffer, offset, current_cmd);
            SolidPassCmd::getInstance()->Size[MAT_NORMAL_MAP] = current_cmd - SolidPassCmd::getInstance()->Offset[MAT_NORMAL_MAP];

            // Grass
            SolidPassCmd::getInstance()->Offset[MAT_GRASS] = current_cmd;
            FillInstancesGrass(MeshForSolidPass[MAT_GRASS], ListInstancedMatGrass::getInstance(), InstanceBuffer, CmdBuffer, offset, current_cmd);
            SolidPassCmd::getInstance()->Size[MAT_GRASS] = current_cmd - SolidPassCmd::getInstance()->Offset[MAT_GRASS];
        }

#pragma omp section
        {
            irr_driver->setPhase(SHADOW_PASS);

            size_t offset = 0, current_cmd = 0;
            std::vector<std::pair<size_t, size_t> > Result;

            for (unsigned i = 0; i < 4; i++)
            {
                // Mat default
                ShadowPassCmd::getInstance()->Offset[i][MAT_DEFAULT] = current_cmd;
                FillInstances(MeshForShadowPass[i][MAT_DEFAULT], ListInstancedMatDefault::getInstance(), ShadowInstanceBuffer, ShadowCmdBuffer, offset, current_cmd);
                ShadowPassCmd::getInstance()->Size[i][MAT_DEFAULT] = current_cmd - ShadowPassCmd::getInstance()->Offset[i][MAT_DEFAULT];

                // Mat AlphaRef
                ShadowPassCmd::getInstance()->Offset[i][MAT_ALPHA_REF] = current_cmd;
                FillInstances(MeshForShadowPass[i][MAT_ALPHA_REF], ListInstancedMatAlphaRef::getInstance(), ShadowInstanceBuffer, ShadowCmdBuffer, offset, current_cmd);
                ShadowPassCmd::getInstance()->Size[i][MAT_ALPHA_REF] = current_cmd - ShadowPassCmd::getInstance()->Offset[i][MAT_ALPHA_REF];

                // Mat Unlit
                ShadowPassCmd::getInstance()->Offset[i][MAT_UNLIT] = current_cmd;
                FillInstances(MeshForShadowPass[i][MAT_UNLIT], ListInstancedMatUnlit::getInstance(), ShadowInstanceBuffer, ShadowCmdBuffer, offset, current_cmd);
                ShadowPassCmd::getInstance()->Size[i][MAT_UNLIT] = current_cmd - ShadowPassCmd::getInstance()->Offset[i][MAT_UNLIT];

                // Mat NormalMap
                ShadowPassCmd::getInstance()->Offset[i][MAT_NORMAL_MAP] = current_cmd;
                FillInstances(MeshForShadowPass[i][MAT_NORMAL_MAP], ListInstancedMatNormalMap::getInstance(), ShadowInstanceBuffer, ShadowCmdBuffer, offset, current_cmd);
                ShadowPassCmd::getInstance()->Size[i][MAT_NORMAL_MAP] = current_cmd - ShadowPassCmd::getInstance()->Offset[i][MAT_NORMAL_MAP];

                // Mat Spheremap
                ShadowPassCmd::getInstance()->Offset[i][MAT_SPHEREMAP] = current_cmd;
                FillInstances(MeshForShadowPass[i][MAT_SPHEREMAP], ListInstancedMatSphereMap::getInstance(), ShadowInstanceBuffer, ShadowCmdBuffer, offset, current_cmd);
                ShadowPassCmd::getInstance()->Size[i][MAT_SPHEREMAP] = current_cmd - ShadowPassCmd::getInstance()->Offset[i][MAT_SPHEREMAP];

                // Mat Detail
                ShadowPassCmd::getInstance()->Offset[i][MAT_DETAIL] = current_cmd;
                FillInstances(MeshForShadowPass[i][MAT_DETAIL], ListInstancedMatDetails::getInstance(), ShadowInstanceBuffer, ShadowCmdBuffer, offset, current_cmd);
                ShadowPassCmd::getInstance()->Size[i][MAT_DETAIL] = current_cmd - ShadowPassCmd::getInstance()->Offset[i][MAT_DETAIL];

                // Mat Grass
                ShadowPassCmd::getInstance()->Offset[i][MAT_GRASS] = current_cmd;
                FillInstancesGrass(MeshForShadowPass[i][MAT_GRASS], ListInstancedMatGrass::getInstance(), ShadowInstanceBuffer, ShadowCmdBuffer, offset, current_cmd);
                ShadowPassCmd::getInstance()->Size[i][MAT_GRASS] = current_cmd - ShadowPassCmd::getInstance()->Offset[i][MAT_GRASS];
            }
        }
    }

    glMemoryBarrier(GL_CLIENT_MAPPED_BUFFER_BARRIER_BIT);
}


void IrrDriver::compressShadowsDrawCalls()
{
/*    ListInstancedMatDefault::getInstance()->clear();
    ListInstancedMatAlphaRef::getInstance()->clear();
    ListInstancedMatGrass::getInstance()->clear();
    ListInstancedMatNormalMap::getInstance()->clear();
    ListInstancedMatSphereMap::getInstance()->clear();
    ListInstancedMatDetails::getInstance()->clear();
    ListInstancedMatUnlit::getInstance()->clear();*/




//    glMemoryBarrier(GL_CLIENT_MAPPED_BUFFER_BARRIER_BIT);
}