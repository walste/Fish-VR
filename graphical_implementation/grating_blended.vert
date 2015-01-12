/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#version 120

varying vec3 cyl_pos;

void main(void)
{
  gl_Position=ftransform(); // standard OpenGL vertex transform
  cyl_pos = gl_Vertex.xyz;
}
