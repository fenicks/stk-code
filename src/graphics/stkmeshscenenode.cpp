#include "stkmeshscenenode.hpp"
#include "stkmesh.hpp"
#include "graphics/irr_driver.hpp"
#include "tracks/track.hpp"
#include <ISceneManager.h>
#include <IMaterialRenderer.h>
#include "config/user_config.hpp"
#include "graphics/callbacks.hpp"
#include "utils/helpers.hpp"
#include "graphics/camera.hpp"
#include "modes/world.hpp"

STKMeshSceneNode::STKMeshSceneNode(irr::scene::IMesh* mesh, ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id,
    const irr::core::vector3df& position,
    const irr::core::vector3df& rotation,
    const irr::core::vector3df& scale) :
    CMeshSceneNode(mesh, parent, mgr, id, position, rotation, scale)
{
    reload_each_frame = false;
    createGLMeshes();
}

void STKMeshSceneNode::setReloadEachFrame(bool val)
{
    reload_each_frame = val;
}

void STKMeshSceneNode::createGLMeshes()
{
    for (u32 i = 0; i<Mesh->getMeshBufferCount(); ++i)
    {
        scene::IMeshBuffer* mb = Mesh->getMeshBuffer(i);
        GLmeshes.push_back(allocateMeshBuffer(mb));
    }
    isMaterialInitialized = false;
}

void STKMeshSceneNode::setFirstTimeMaterial()
{
  if (isMaterialInitialized)
      return;
  irr::video::IVideoDriver* driver = irr_driver->getVideoDriver();
  for (u32 i = 0; i<Mesh->getMeshBufferCount(); ++i)
  {
      scene::IMeshBuffer* mb = Mesh->getMeshBuffer(i);
      if (!mb)
        continue;
      video::E_MATERIAL_TYPE type = mb->getMaterial().MaterialType;
      f32 MaterialTypeParam = mb->getMaterial().MaterialTypeParam;
      video::IMaterialRenderer* rnd = driver->getMaterialRenderer(type);
      if (!isObject(type))
      {
#ifdef DEBUG
          Log::warn("material", "Unhandled (static) material type : %d", type);
#endif
          continue;
      }

      GLMesh &mesh = GLmeshes[i];
      if (rnd->isTransparent())
      {
          TransparentMaterial TranspMat = MaterialTypeToTransparentMaterial(type, MaterialTypeParam);
          initvaostate(mesh, TranspMat);
          TransparentMesh[TranspMat].push_back(&mesh);
      }
      else
      {
          GeometricMaterial GeometricType = MaterialTypeToGeometricMaterial(type);
          ShadedMaterial ShadedType = MaterialTypeToShadedMaterial(type, mesh.textures);
          initvaostate(mesh, GeometricType, ShadedType);
          GeometricMesh[GeometricType].push_back(&mesh);
          ShadedMesh[ShadedType].push_back(&mesh);
      }
  }
  isMaterialInitialized = true;
}

void STKMeshSceneNode::cleanGLMeshes()
{
    for (u32 i = 0; i < GLmeshes.size(); ++i)
    {
        GLMesh mesh = GLmeshes[i];
        if (!mesh.vertex_buffer)
            continue;
        if (mesh.vao)
            glDeleteVertexArrays(1, &(mesh.vao));
        glDeleteBuffers(1, &(mesh.vertex_buffer));
        glDeleteBuffers(1, &(mesh.index_buffer));
    }
    GLmeshes.clear();
    for (unsigned i = 0; i < FPSM_COUNT; i++)
        GeometricMesh[i].clearWithoutDeleting();
    for (unsigned i = 0; i < SM_COUNT; i++)
        ShadedMesh[i].clearWithoutDeleting();
}

void STKMeshSceneNode::setMesh(irr::scene::IMesh* mesh)
{
    CMeshSceneNode::setMesh(mesh);
    cleanGLMeshes();
    createGLMeshes();
}

STKMeshSceneNode::~STKMeshSceneNode()
{
    cleanGLMeshes();
}

void STKMeshSceneNode::drawGlow(const GLMesh &mesh)
{
    ColorizeProvider * const cb = (ColorizeProvider *)irr_driver->getCallback(ES_COLORIZE);

    GLenum ptype = mesh.PrimitiveType;
    GLenum itype = mesh.IndexType;
    size_t count = mesh.IndexCount;

    MeshShader::ColorizeShader::setUniforms(AbsoluteTransformation, cb->getRed(), cb->getGreen(), cb->getBlue());

    assert(mesh.vao);
    glBindVertexArray(mesh.vao);
    glDrawElements(ptype, count, itype, 0);
}

static video::ITexture *displaceTex = 0;

void STKMeshSceneNode::drawDisplace(const GLMesh &mesh)
{
    DisplaceProvider * const cb = (DisplaceProvider *)irr_driver->getCallback(ES_DISPLACE);

    GLenum ptype = mesh.PrimitiveType;
    GLenum itype = mesh.IndexType;
    size_t count = mesh.IndexCount;

    // Generate displace mask
    // Use RTT_TMP4 as displace mask
    irr_driver->getFBO(FBO_TMP1_WITH_DS).Bind();

    glUseProgram(MeshShader::DisplaceMaskShader::Program);
    MeshShader::DisplaceMaskShader::setUniforms(AbsoluteTransformation);

    glBindVertexArray(mesh.vao);
    glDrawElements(ptype, count, itype, 0);

    // Render the effect
    if (!displaceTex)
        displaceTex = irr_driver->getTexture(FileManager::TEXTURE, "displace.png");
    irr_driver->getFBO(FBO_DISPLACE).Bind();
    setTexture(0, getTextureGLuint(displaceTex), GL_LINEAR, GL_LINEAR, true);
    setTexture(1, irr_driver->getRenderTargetTexture(RTT_TMP1), GL_LINEAR, GL_LINEAR, true);
    setTexture(2, irr_driver->getRenderTargetTexture(RTT_COLOR), GL_LINEAR, GL_LINEAR, true);
    glUseProgram(MeshShader::DisplaceShader::Program);
    MeshShader::DisplaceShader::setUniforms(AbsoluteTransformation,
                                            core::vector2df(cb->getDirX(), cb->getDirY()),
                                            core::vector2df(cb->getDir2X(), cb->getDir2Y()),
                                            core::vector2df(float(UserConfigParams::m_width),
                                                            float(UserConfigParams::m_height)),
                                            0, 1, 2);

    glDrawElements(ptype, count, itype, 0);
}

void STKMeshSceneNode::drawTransparent(const GLMesh &mesh, video::E_MATERIAL_TYPE type)
{
    assert(irr_driver->getPhase() == TRANSPARENT_PASS);

    ModelViewProjectionMatrix = computeMVP(AbsoluteTransformation);

    if (type == irr_driver->getShader(ES_BUBBLES))
        drawBubble(mesh, ModelViewProjectionMatrix);
//    else if (World::getWorld()->getTrack()->isFogEnabled())
//        drawTransparentFogObject(mesh, ModelViewProjectionMatrix, TextureMatrix);
    else
        drawTransparentObject(mesh, ModelViewProjectionMatrix, mesh.TextureMatrix);
    return;
}

void STKMeshSceneNode::drawSolidPass1(const GLMesh &mesh, GeometricMaterial type)
{
    irr_driver->IncreaseObjectCount();
    windDir = getWind();
    switch (type)
    {
    case FPSM_GRASS:
        drawGrassPass1(mesh, ModelViewProjectionMatrix, TransposeInverseModelView, windDir);
        break;
    default:
        assert(0 && "wrong geometric material");
    }
}

void STKMeshSceneNode::drawSolidPass2(const GLMesh &mesh, ShadedMaterial type)
{
    switch (type)
    {
    case SM_GRASS:
        drawGrassPass2(mesh, ModelViewProjectionMatrix, windDir);
        break;
    default:
        assert(0 && "Wrong shaded material");
        break;
    }
}

void STKMeshSceneNode::updatevbo()
{
    for (unsigned i = 0; i < Mesh->getMeshBufferCount(); ++i)
    {
        scene::IMeshBuffer* mb = Mesh->getMeshBuffer(i);
        if (!mb)
            continue;
        GLMesh &mesh = GLmeshes[i];
        glBindVertexArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, mesh.vertex_buffer);
        const void* vertices = mb->getVertices();
        const u32 vertexCount = mb->getVertexCount();
        const c8* vbuf = static_cast<const c8*>(vertices);
        glBufferData(GL_ARRAY_BUFFER, vertexCount * mesh.Stride, vbuf, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.index_buffer);
        const void* indices = mb->getIndices();
        mesh.IndexCount = mb->getIndexCount();
        GLenum indexSize;
        switch (mb->getIndexType())
        {
        case irr::video::EIT_16BIT:
            indexSize = sizeof(u16);
            break;
        case irr::video::EIT_32BIT:
            indexSize = sizeof(u32);
            break;
        default:
            assert(0 && "Wrong index size");
        }
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.IndexCount * indexSize, indices, GL_STATIC_DRAW);
    }
}

void STKMeshSceneNode::render()
{
    irr::video::IVideoDriver* driver = irr_driver->getVideoDriver();

    if (!Mesh || !driver)
        return;

    if (reload_each_frame)
        updatevbo();

    bool isTransparentPass =
        SceneManager->getSceneNodeRenderPass() == scene::ESNRP_TRANSPARENT;

    ++PassCount;

    Box = Mesh->getBoundingBox();

    setFirstTimeMaterial();

    for (u32 i = 0; i < Mesh->getMeshBufferCount(); ++i)
    {
        scene::IMeshBuffer* mb = Mesh->getMeshBuffer(i);
        if (!mb)
            continue;
        GLmeshes[i].TextureMatrix = getMaterial(i).getTextureMatrix(0);
    }

    if (irr_driver->getPhase() == SOLID_NORMAL_AND_DEPTH_PASS || irr_driver->getPhase() == SHADOW_PASS)
    {
        if (reload_each_frame)
            glDisable(GL_CULL_FACE);
        ModelViewProjectionMatrix = computeMVP(AbsoluteTransformation);
        TransposeInverseModelView = computeTIMV(AbsoluteTransformation);
        core::matrix4 invmodel;
        AbsoluteTransformation.getInverse(invmodel);
        GLMesh* mesh;
        for_in(mesh, GeometricMesh[FPSM_DEFAULT])
        {
            GroupedFPSM<FPSM_DEFAULT>::MeshSet.push_back(mesh);
            GroupedFPSM<FPSM_DEFAULT>::MVPSet.push_back(AbsoluteTransformation);
            GroupedFPSM<FPSM_DEFAULT>::TIMVSet.push_back(invmodel);
        }

        for_in(mesh, GeometricMesh[FPSM_ALPHA_REF_TEXTURE])
        {
            GroupedFPSM<FPSM_ALPHA_REF_TEXTURE>::MeshSet.push_back(mesh);
            GroupedFPSM<FPSM_ALPHA_REF_TEXTURE>::MVPSet.push_back(AbsoluteTransformation);
            GroupedFPSM<FPSM_ALPHA_REF_TEXTURE>::TIMVSet.push_back(invmodel);
        }

        for_in(mesh, GeometricMesh[FPSM_NORMAL_MAP])
        {
            GroupedFPSM<FPSM_NORMAL_MAP>::MeshSet.push_back(mesh);
            GroupedFPSM<FPSM_NORMAL_MAP>::MVPSet.push_back(AbsoluteTransformation);
            GroupedFPSM<FPSM_NORMAL_MAP>::TIMVSet.push_back(invmodel);
        }

        if (irr_driver->getPhase() == SOLID_NORMAL_AND_DEPTH_PASS)
        {
            if (!GeometricMesh[FPSM_GRASS].empty())
                glUseProgram(MeshShader::GrassPass1Shader::Program);
            for_in(mesh, GeometricMesh[FPSM_GRASS])
                drawSolidPass1(*mesh, FPSM_GRASS);
        }

        if (reload_each_frame)
            glEnable(GL_CULL_FACE);
        return;
    }

    if (irr_driver->getPhase() == SOLID_LIT_PASS)
    {
        if (reload_each_frame)
            glDisable(GL_CULL_FACE);

        core::matrix4 invmodel;
        AbsoluteTransformation.getInverse(invmodel);

        GLMesh* mesh;
        for_in(mesh, ShadedMesh[SM_DEFAULT])
        {
            GroupedSM<SM_DEFAULT>::MeshSet.push_back(mesh);
            GroupedSM<SM_DEFAULT>::MVPSet.push_back(AbsoluteTransformation);
            GroupedSM<SM_DEFAULT>::TIMVSet.push_back(invmodel);
        }

        for_in(mesh, ShadedMesh[SM_ALPHA_REF_TEXTURE])
        {
            GroupedSM<SM_ALPHA_REF_TEXTURE>::MeshSet.push_back(mesh);
            GroupedSM<SM_ALPHA_REF_TEXTURE>::MVPSet.push_back(AbsoluteTransformation);
            GroupedSM<SM_ALPHA_REF_TEXTURE>::TIMVSet.push_back(invmodel);
        }

        for_in(mesh, ShadedMesh[SM_RIMLIT])
        {
            GroupedSM<SM_RIMLIT>::MeshSet.push_back(mesh);
            GroupedSM<SM_RIMLIT>::MVPSet.push_back(AbsoluteTransformation);
            GroupedSM<SM_RIMLIT>::TIMVSet.push_back(invmodel);
        }


        for_in(mesh, ShadedMesh[SM_SPHEREMAP])
        {
            GroupedSM<SM_SPHEREMAP>::MeshSet.push_back(mesh);
            GroupedSM<SM_SPHEREMAP>::MVPSet.push_back(AbsoluteTransformation);
            GroupedSM<SM_SPHEREMAP>::TIMVSet.push_back(invmodel);
        }

        for_in(mesh, ShadedMesh[SM_SPLATTING])
        {
            GroupedSM<SM_SPLATTING>::MeshSet.push_back(mesh);
            GroupedSM<SM_SPLATTING>::MVPSet.push_back(AbsoluteTransformation);
            GroupedSM<SM_SPLATTING>::TIMVSet.push_back(invmodel);
        }

        for_in(mesh, ShadedMesh[SM_UNLIT])
        {
            GroupedSM<SM_UNLIT>::MeshSet.push_back(mesh);
            GroupedSM<SM_UNLIT>::MVPSet.push_back(AbsoluteTransformation);
            GroupedSM<SM_UNLIT>::TIMVSet.push_back(invmodel);
        }

        for_in(mesh, ShadedMesh[SM_DETAILS])
        {
            GroupedSM<SM_DETAILS>::MeshSet.push_back(mesh);
            GroupedSM<SM_DETAILS>::MVPSet.push_back(AbsoluteTransformation);
            GroupedSM<SM_DETAILS>::TIMVSet.push_back(invmodel);
        }

        if (!ShadedMesh[SM_GRASS].empty())
            glUseProgram(MeshShader::GrassPass2Shader::Program);
        for_in(mesh, ShadedMesh[SM_GRASS])
            drawSolidPass2(*mesh, SM_GRASS);

        if (reload_each_frame)
            glEnable(GL_CULL_FACE);
        return;
    }

    if (irr_driver->getPhase() == GLOW_PASS)
    {
        glUseProgram(MeshShader::ColorizeShader::Program);
        for (u32 i = 0; i < Mesh->getMeshBufferCount(); ++i)
        {
            scene::IMeshBuffer* mb = Mesh->getMeshBuffer(i);
            if (!mb)
                continue;
            drawGlow(GLmeshes[i]);
        }
    }

    if (irr_driver->getPhase() == TRANSPARENT_PASS)
    {
        ModelViewProjectionMatrix = computeMVP(AbsoluteTransformation);

        GLMesh* mesh;

        for_in(mesh, TransparentMesh[TM_DEFAULT])
        {
            TransparentMeshes<TM_DEFAULT>::MeshSet.push_back(mesh);
            TransparentMeshes<TM_DEFAULT>::MVPSet.push_back(AbsoluteTransformation);
        }

        for_in(mesh, TransparentMesh[TM_ADDITIVE])
        {
            TransparentMeshes<TM_ADDITIVE>::MeshSet.push_back(mesh);
            TransparentMeshes<TM_ADDITIVE>::MVPSet.push_back(AbsoluteTransformation);
        }

        if (!TransparentMesh[TM_BUBBLE].empty())
            glUseProgram(MeshShader::BubbleShader::Program);
        for_in(mesh, TransparentMesh[TM_BUBBLE])
            drawBubble(*mesh, ModelViewProjectionMatrix);
        return;
    }

    if (irr_driver->getPhase() == DISPLACEMENT_PASS)
    {
        for (u32 i = 0; i < Mesh->getMeshBufferCount(); ++i)
        {
            scene::IMeshBuffer* mb = Mesh->getMeshBuffer(i);
            if (!mb)
                continue;
            drawDisplace(GLmeshes[i]);
        }
    }
}
