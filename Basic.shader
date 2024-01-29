#shader vertex
#version 330 core

uniform vec2 translation;

layout(location = 0) in vec4 position;

void main()
{
    gl_Position = vec4(position.xy + translation, position.zw);
};

#shader fragment
#version 330 core

layout(location = 0) out vec4 color;

uniform vec3 inputCol;


void main()
{
    color = vec4(inputCol, 1.0);
};