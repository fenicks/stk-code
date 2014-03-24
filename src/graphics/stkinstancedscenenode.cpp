#include "stkinstancedscenenode.hpp"
#include "graphics/irr_driver.hpp"

STKInstancedSceneNode::STKInstancedSceneNode(irr::scene::IMesh* mesh, ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id,
    const irr::core::vector3df& position,
    const irr::core::vector3df& rotation,
    const irr::core::vector3df& scale) :
    CMeshSceneNode(mesh, parent, mgr, id, position, rotation, scale)
{
    createGLMeshes();
    setAutomaticCulling(0);
}

void STKInstancedSceneNode::createGLMeshes()
{
    for (u32 i = 0; i<Mesh->getMeshBufferCount(); ++i)
    {
        scene::IMeshBuffer* mb = Mesh->getMeshBuffer(i);
        GLmeshes.push_back(allocateMeshBuffer(mb));
    }
    isMaterialInitialized = false;
}

void STKInstancedSceneNode::initinstancedvaostate(GLMesh &mesh, GeometricMaterial GeoMat, ShadedMaterial ShadedMat)
{
    switch (GeoMat)
    {
    case FPSM_DEFAULT:
        mesh.vao_first_pass = createVAO(mesh.vertex_buffer, mesh.index_buffer,
            MeshShader::InstancedObjectPass1Shader::attrib_position, -1, -1, MeshShader::InstancedObjectPass1Shader::attrib_normal, -1, -1, -1, mesh.Stride);
        break;
    case FPSM_GRASS:
        mesh.vao_first_pass = createVAO(mesh.vertex_buffer, mesh.index_buffer,
            MeshShader::InstancedGrassPass1Shader::attrib_position, MeshShader::InstancedGrassPass1Shader::attrib_texcoord, -1, MeshShader::InstancedGrassPass1Shader::attrib_normal, -1, -1, MeshShader::InstancedGrassPass1Shader::attrib_color, mesh.Stride);
        break;
    default:
      return;
    }

    glGenBuffers(1, &instances_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, instances_vbo);
    glBufferData(GL_ARRAY_BUFFER, instance_pos.size() * sizeof(float), instance_pos.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(MeshShader::InstancedObjectPass1Shader::attrib_origin);
    glVertexAttribPointer(MeshShader::InstancedObjectPass1Shader::attrib_origin, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    glVertexAttribDivisor(MeshShader::InstancedObjectPass1Shader::attrib_origin, 1);

    switch (ShadedMat)
    {
    case SM_DEFAULT:
        mesh.vao_second_pass = createVAO(mesh.vertex_buffer, mesh.index_buffer,
            MeshShader::InstancedObjectPass2Shader::attrib_position, MeshShader::InstancedObjectPass2Shader::attrib_texcoord, -1, -1, -1, -1, -1, mesh.Stride);
        break;
    case SM_GRASS:
        mesh.vao_second_pass = createVAO(mesh.vertex_buffer, mesh.index_buffer,
            MeshShader::InstancedGrassPass2Shader::attrib_position, MeshShader::InstancedGrassPass2Shader::attrib_texcoord, -1, -1, -1, -1, MeshShader::InstancedGrassPass2Shader::attrib_color, mesh.Stride);
        break;
    default:
      return;
    }

    glBindBuffer(GL_ARRAY_BUFFER, instances_vbo);
    glEnableVertexAttribArray(MeshShader::InstancedObjectPass2Shader::attrib_origin);
    glVertexAttribPointer(MeshShader::InstancedObjectPass2Shader::attrib_origin, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    glVertexAttribDivisor(MeshShader::InstancedObjectPass2Shader::attrib_origin, 1);

    glBindVertexArray(0);
}

void STKInstancedSceneNode::setFirstTimeMaterial()
{
    if (isMaterialInitialized)
        return;
    for (u32 i = 0; i<Mesh->getMeshBufferCount(); ++i)
    {
        scene::IMeshBuffer* mb = Mesh->getMeshBuffer(i);
        if (!mb)
            continue;
        video::E_MATERIAL_TYPE type = mb->getMaterial().MaterialType;

        GLMesh &mesh = GLmeshes[i];
        GeometricMaterial GeometricType = MaterialTypeToGeometricMaterial(type);
        ShadedMaterial ShadedType = MaterialTypeToShadedMaterial(type, mesh.textures);
        initinstancedvaostate(mesh, GeometricType, ShadedType);
        if (mesh.vao_first_pass)
            GeometricMesh[GeometricType].push_back(&mesh);
        if (mesh.vao_second_pass)
            ShadedMesh[ShadedType].push_back(&mesh);
    }
    isMaterialInitialized = true;
}

void STKInstancedSceneNode::addWorldMatrix(const core::vector3df &v)
{
    instance_pos.push_back(v.X);
    instance_pos.push_back(v.Y);
    instance_pos.push_back(v.Z);
}

static void drawFSPMDefault(GLMesh &mesh, const core::matrix4 &ModelViewProjectionMatrix, size_t instance_count)
{
  irr_driver->IncreaseObjectCount();
  GLenum ptype = mesh.PrimitiveType;
  GLenum itype = mesh.IndexType;
  size_t count = mesh.IndexCount;

  MeshShader::InstancedObjectPass1Shader::setUniforms(ModelViewProjectionMatrix, irr_driver->getVideoDriver()->getTransform(video::ETS_VIEW));

  glBindVertexArray(mesh.vao_first_pass);
  glDrawElementsInstanced(ptype, count, itype, 0, instance_count);
}

static void drawFSPMGrass(GLMesh &mesh, const core::matrix4 &ModelViewProjectionMatrix, const core::vector3df &windDir, size_t instance_count)
{
    irr_driver->IncreaseObjectCount();
    GLenum ptype = mesh.PrimitiveType;
    GLenum itype = mesh.IndexType;
    size_t count = mesh.IndexCount;

    setTexture(0, mesh.textures[0], GL_LINEAR, GL_LINEAR_MIPMAP_LINEAR, true);
    MeshShader::InstancedGrassPass1Shader::setUniforms(ModelViewProjectionMatrix, irr_driver->getVideoDriver()->getTransform(video::ETS_VIEW), windDir, 0);

    glBindVertexArray(mesh.vao_first_pass);
    glDrawElementsInstanced(ptype, count, itype, 0, instance_count);
}

static void drawSMDefault(GLMesh &mesh, const core::matrix4 &ModelViewProjectionMatrix, size_t instance_count)
{
  irr_driver->IncreaseObjectCount();
  GLenum ptype = mesh.PrimitiveType;
  GLenum itype = mesh.IndexType;
  size_t count = mesh.IndexCount;

  setTexture(MeshShader::InstancedObjectPass2Shader::TU_Albedo, mesh.textures[0], GL_LINEAR, GL_LINEAR_MIPMAP_LINEAR, true);

  MeshShader::InstancedObjectPass2Shader::setUniforms(ModelViewProjectionMatrix, core::matrix4::EM4CONST_IDENTITY);

  glBindVertexArray(mesh.vao_second_pass);
  glDrawElementsInstanced(ptype, count, itype, 0, instance_count);
}

static void drawSMGrass(GLMesh &mesh, const core::matrix4 &ModelViewProjectionMatrix, const core::vector3df &windDir, size_t instance_count)
{
    irr_driver->IncreaseObjectCount();
    GLenum ptype = mesh.PrimitiveType;
    GLenum itype = mesh.IndexType;
    size_t count = mesh.IndexCount;

    setTexture(MeshShader::InstancedGrassPass2Shader::TU_Albedo, mesh.textures[0], GL_LINEAR, GL_LINEAR_MIPMAP_LINEAR, true);

    MeshShader::InstancedGrassPass2Shader::setUniforms(ModelViewProjectionMatrix, windDir);

    glBindVertexArray(mesh.vao_second_pass);
    glDrawElementsInstanced(ptype, count, itype, 0, instance_count);
}

void STKInstancedSceneNode::render()
{
    setFirstTimeMaterial();

    if (irr_driver->getPhase() == SOLID_NORMAL_AND_DEPTH_PASS)
    {
        ModelViewProjectionMatrix = irr_driver->getVideoDriver()->getTransform(video::ETS_PROJECTION);
        ModelViewProjectionMatrix *= irr_driver->getVideoDriver()->getTransform(video::ETS_VIEW);

        if (!GeometricMesh[FPSM_DEFAULT].empty())
            glUseProgram(MeshShader::InstancedObjectPass1Shader::Program);
        for (unsigned i = 0; i < GeometricMesh[FPSM_DEFAULT].size(); i++)
            drawFSPMDefault(*GeometricMesh[FPSM_DEFAULT][i], ModelViewProjectionMatrix, instance_pos.size() / 3);

        windDir = getWind();
        if (!GeometricMesh[FPSM_GRASS].empty())
            glUseProgram(MeshShader::InstancedGrassPass1Shader::Program);
        for (unsigned i = 0; i < GeometricMesh[FPSM_GRASS].size(); i++)
            drawFSPMGrass(*GeometricMesh[FPSM_GRASS][i], ModelViewProjectionMatrix, windDir, instance_pos.size() / 3);
        return;
    }

    if (irr_driver->getPhase() == SOLID_LIT_PASS)
    {
        if (!ShadedMesh[SM_DEFAULT].empty())
            glUseProgram(MeshShader::InstancedObjectPass2Shader::Program);
        for (unsigned i = 0; i < ShadedMesh[FPSM_DEFAULT].size(); i++)
            drawSMDefault(*ShadedMesh[FPSM_DEFAULT][i], ModelViewProjectionMatrix, instance_pos.size() / 3);

        if (!ShadedMesh[SM_GRASS].empty())
            glUseProgram(MeshShader::InstancedGrassPass2Shader::Program);
        for (unsigned i = 0; i < ShadedMesh[SM_GRASS].size(); i++)
            drawSMGrass(*ShadedMesh[SM_GRASS][i], ModelViewProjectionMatrix, windDir, instance_pos.size() / 3);
        return;
    }
}