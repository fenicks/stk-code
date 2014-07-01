uniform sampler2D tex;

in vec2 uv;
in vec4 color;
out vec4 FragColor;

void main(void)
{
    vec4 col = texture(tex, uv);
    if (col.a < 0.5) discard;
    FragColor = vec4(col.xyz * color.xyz, 1.);
}
