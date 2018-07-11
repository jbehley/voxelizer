#version 330 core

layout (location = 0) in vec3  in_position;
layout (location = 1) in uint in_label;

uniform sampler2DRect label_colors;
uniform vec4 custom_color;
uniform bool use_custom_color;

#include "shaders/color.glsl"


out VOXEL {
  vec3 pos;
  vec4 color;
} vs_out;


void main()
{
  vec4 in_color = texture(label_colors, vec2(in_label, 0));
  if (use_custom_color)  in_color = custom_color;
  
  vs_out.pos = in_position;
  vs_out.color = in_color;
}
