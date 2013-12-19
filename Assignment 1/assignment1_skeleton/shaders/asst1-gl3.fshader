#version 130

uniform float uVertexScale;
uniform float uTime;
uniform float uRatio;
uniform sampler2D uTexUnit0, uTexUnit1;

in vec2 vTexCoord0, vTexCoord1;
in vec3 vColor;

out vec4 fragColor;

void main(void) {
  vec4 color = vec4(vColor.x, vColor.y, vColor.z, 1);
  vec4 texColor0 = texture(uTexUnit0, vTexCoord0);
  vec4 texColor1 = texture(uTexUnit1, vTexCoord1);

  // Value ranges from 0 to 1 (.5 + .5*sin(x))
  float lerper = clamp(.5 + 0.5 * sin(uTime), 0.0, 1.0);
  float lerper2 = clamp(uRatio, 0.0, 1.0);

  fragColor = ((lerper)*texColor1 + (1.0-lerper)*texColor0) * lerper2 + color * (1.0-lerper2);
}
