#extension GL_ARB_bindless_texture : enable

#ifdef GL_ARB_bindless_texture
layout(bindless_sampler) uniform sampler2D tex;
#else
uniform sampler2D tex;
#endif

in vec2 uv;
in vec4 color;
out vec4 FragColor;

void main(void)
{
    vec4 col = texture(tex, uv);
    col.xyz *= pow(color.xyz, vec3(2.2));
    if (col.a < 0.5) discard;
    FragColor = vec4(col.xyz, 1.);
}
