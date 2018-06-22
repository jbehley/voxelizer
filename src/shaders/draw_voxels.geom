#version 330 core

layout(points) in;
layout(triangle_strip, max_vertices = 36) out;


in VOXEL {
  vec3 pos;
  vec4 color;
} gs_in[];

uniform mat4 mvp;
uniform vec4 voxelOffset;
uniform float voxelSize;

out vec4 color;
out vec3 position;
out vec3 normal;

void main()
{
   // see https://stackoverflow.com/questions/28375338/cube-using-single-gl-triangle-strip for order.
   // 4 3 7 8 5 3 1 4 2 7 6 5 2 1
   // only 14 vertices instead  of 36! However, I really don't know how to handle there the normals for the lighting.
   
   
    vec4 p = vec4(gs_in[0].pos.xyz, 1);
    color = gs_in[0].color; 
    
    vec4 p1 = p + vec4(voxelSize,0,0,0);
    vec4 p2 = p + vec4(0,0,0,0);
    vec4 p3 = p + vec4(voxelSize,voxelSize,0,0);
    vec4 p4 = p + vec4(0,voxelSize,0,0);
    vec4 p5 = p + vec4(voxelSize,0,voxelSize,0);
    vec4 p6 = p + vec4(0,0,voxelSize,0);
    vec4 p7 = p + vec4(0, voxelSize,voxelSize,0);
    vec4 p8 = p + vec4(voxelSize,voxelSize,voxelSize,0);
    
    normal = vec3(0,1,0);
    position = p4.xyz;
    gl_Position = mvp * p4;
    EmitVertex();   
    position = p3.xyz;
    gl_Position = mvp * p3;
    EmitVertex();  
    position = p7.xyz;
    gl_Position = mvp * p7;
    EmitVertex();
    EndPrimitive();
    
    position = p3.xyz;
    gl_Position = mvp * p3;
    EmitVertex();  
    position = p7.xyz;
    gl_Position = mvp * p7;
    EmitVertex();
    position = p8.xyz;
    gl_Position = mvp * p8;
    EmitVertex();  
    EndPrimitive();
    
    
    normal = vec3(0,0,1);
    position = p7.xyz;
    gl_Position = mvp * p7;
    EmitVertex();
    position = p8.xyz;
    gl_Position = mvp * p8;
    EmitVertex(); 
    position = p5.xyz; 
    gl_Position = mvp * p5;
    EmitVertex();
    EndPrimitive();
    
    position = p7.xyz; 
    gl_Position = mvp * p7;
    EmitVertex();
    position = p6.xyz; 
    gl_Position = mvp * p6;
    EmitVertex();
    position = p5.xyz; 
    gl_Position = mvp * p5;
    EmitVertex();
    EndPrimitive();
    
    
    normal = vec3(1,0,0);
    position = p8.xyz;
    gl_Position = mvp * p8;
    EmitVertex();
    position = p5.xyz;
    gl_Position = mvp * p5;
    EmitVertex();
    position = p3.xyz;
    gl_Position = mvp * p3;
    EmitVertex();
    EndPrimitive();
    
    position = p5.xyz;
    gl_Position = mvp * p5;
    EmitVertex();
    position = p3.xyz;
    gl_Position = mvp * p3;
    EmitVertex();
    position = p1.xyz;
    gl_Position = mvp * p1;
    EmitVertex();
    EndPrimitive();
    
    normal = vec3(0,0,-1);
    position = p3.xyz;
    gl_Position = mvp * p3;
    EmitVertex();
    position = p1.xyz;
    gl_Position = mvp * p1;
    EmitVertex();
    position = p4.xyz;
    gl_Position = mvp * p4;
    EmitVertex(); 
    EndPrimitive();
    
    position = p1.xyz;
    gl_Position = mvp * p1;
    EmitVertex();
    position = p4.xyz;
    gl_Position = mvp * p4;
    EmitVertex();
    position = p2.xyz; 
    gl_Position = mvp * p2;
    EmitVertex(); 
    EndPrimitive();
      
    normal = vec3(-1,0,0);  
    position = p4.xyz; 
    gl_Position = mvp * p4;
    EmitVertex();
    position = p2.xyz; 
    gl_Position = mvp * p2;
    EmitVertex(); 
    position = p7.xyz; 
    gl_Position = mvp * p7;
    EmitVertex();
    EndPrimitive();
    
    position = p2.xyz; 
    gl_Position = mvp * p2;
    EmitVertex(); 
    position = p7.xyz; 
    gl_Position = mvp * p7;
    EmitVertex();
    position = p6.xyz; 
    gl_Position = mvp * p6;
    EmitVertex();
    EndPrimitive();
    
    
    normal = vec3(0,-1,0);
    position = p6.xyz;
    gl_Position = mvp * p6;
    EmitVertex();
    position = p5.xyz;
    gl_Position = mvp * p5;
    EmitVertex();
    position = p2.xyz;
    gl_Position = mvp * p2;
    EmitVertex(); 
    EndPrimitive();
    
    position = p5.xyz;
    gl_Position = mvp * p5;
    EmitVertex();
    position = p2.xyz;
    gl_Position = mvp * p2;
    EmitVertex();
    position = p1.xyz; 
    gl_Position = mvp * p1;
    EmitVertex(); 
    EndPrimitive();
 
}