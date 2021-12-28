package samples;

import ewbik.processing.Armature;
import ewbik.processing.sceneGraph.Axes;
import ewbik.processing.singlePrecision.Bone;
import ewbik.processing.singlePrecision.IKPin;
import ewbik.processing.singlePrecision.Kusudama;
import processing.core.PApplet;
import processing.core.PConstants;
import processing.core.PGraphics;
import processing.core.PVector;
import processing.opengl.PShader;

import java.io.FileWriter;
import java.io.IOException;

public class UI {
    PApplet pa;
    PGraphics display, stencil;
    PShader blurshader;

    public UI(PApplet p) {
        try {
            FileWriter writer = new FileWriter("blur-sep.glsl");
            writer.write("// Adapted from:\r\n// http://callumhay.blogspot.com/2010/09/gaussian-blur-shader-glsl.html\r\n// Original shader available as part of the processing library under the \"SepBlur\" demo.\r\n// Modifications made by Eron Gjoni for proper handling of gamma (see: https://www.youtube.com/watch?v=LKnqECcg6Gw)\r\n// and (rudimentary) stencil masking suppor. \r\n\r\n\r\n#ifdef GL_ES\r\nprecision mediump float;\r\nprecision mediump int;\r\n#endif\r\n\r\n#define PROCESSING_TEXTURE_SHADER\r\n\r\nuniform sampler2D texture;\r\nuniform sampler2D mask;\r\n\r\n// The inverse of the texture dimensions along X and Y\r\nuniform vec2 texOffset;\r\n\r\nvarying vec4 vertColor;\r\nvarying vec4 vertTexCoord;\r\n\r\nuniform int blurSize;       \r\nuniform int horizontalPass; // 0 or 1 to indicate vertical or horizontal pass\r\nuniform float sigma;        // The sigma value for the gaussian function: higher value means more blur\r\n                            // A good value for 9x9 is around 3 to 5\r\n                            // A good value for 7x7 is around 2.5 to 4\r\n                            // A good value for 5x5 is around 2 to 3.5\r\n                            // ... play around with this based on what you need :)\r\n\r\nconst float pi = 3.14159265;\r\n\r\nbool isZero(in vec3 v) {\r\n\treturn v.x+v.y+v.z == 0.0; \r\n}\r\n\r\nbool isZero(in vec4 v) {\r\n\treturn v.w+v.x+v.y+v.z == 0.0; \r\n}\r\n\r\nbool isWorthy(vec4 center, vec4 contender) {\r\n\treturn center == contender;\r\n}\r\n\r\nvoid main() {  \r\n\r\n    float numBlurPixelsPerSide = float(blurSize / 2); \r\n    vec2 blurMultiplyVec = 0 < horizontalPass ? vec2(1.0, 0.0) : vec2(0.0, 1.0);\r\n\r\n    // Incremental Gaussian Coefficent Calculation (See GPU Gems 3 pp. 877 - 889)\r\n    vec3 incrementalGaussian;\r\n    incrementalGaussian.x = 1.0 / (sqrt(2.0 * pi) * sigma);\r\n    incrementalGaussian.y = exp(-0.5 / (sigma * sigma));\r\n    incrementalGaussian.z = incrementalGaussian.y * incrementalGaussian.y;\r\n\r\n    vec4 avgValue = vec4(0.0, 0.0, 0.0, 0.0);\r\n    float coefficientSum = 0.0;\r\n\r\n    // Take the central sample first...\r\n    avgValue = texture2D(texture, vertTexCoord.st);\r\n    vec4 baseMask = texture2D(mask, vertTexCoord.st);\r\n    if(!isZero(baseMask.rgb)) {\r\n        avgValue *= avgValue;\r\n        avgValue = avgValue * incrementalGaussian.x;\r\n        coefficientSum += incrementalGaussian.x;\r\n        incrementalGaussian.xy *= incrementalGaussian.yz;\r\n\r\n        // Go through the remaining 8 vertical samples (4 on each side of the center)\r\n        for (float i = 1.0; i <= numBlurPixelsPerSide; i++) { \r\n        \tvec4 maskL = texture2D(mask, vertTexCoord.st - i * texOffset *  blurMultiplyVec); \r\n                                    \r\n            if(isWorthy(baseMask, maskL) && !isZero(maskL.rgb)) {\r\n            \r\n            \tvec4 left = texture2D(texture, vertTexCoord.st - i * texOffset * \r\n                                    blurMultiplyVec, 0);\r\n                left *= left;\r\n                left = left * incrementalGaussian.x;\r\n                avgValue += left;  \r\n                coefficientSum += incrementalGaussian.x;\r\n            } else {\r\n            \t//discard;\r\n            }            \r\n            \r\n            vec4 maskR = texture2D(mask, vertTexCoord.st + i * texOffset * \r\n                                    blurMultiplyVec);\r\n            \r\n            if(isWorthy(baseMask, maskR) && !isZero(maskR.rgb)) {\r\n            \r\n            \tvec4 right = texture2D(texture, vertTexCoord.st + i * texOffset * \r\n                                    blurMultiplyVec);\r\n                right *= right;\r\n                right = right * incrementalGaussian.x; \r\n                avgValue +=  right;\r\n                coefficientSum += incrementalGaussian.x;\r\n            } else {\r\n            \t//discard;\r\n            }\r\n            \r\n            \r\n            incrementalGaussian.xy *= incrementalGaussian.yz;\r\n        }\r\n    \r\n        avgValue /= coefficientSum;\r\n        avgValue = sqrt(avgValue); \r\n    }\r\n\r\n  gl_FragColor = avgValue;\r\n}");
            writer.close();
            writer = new FileWriter("kusudama_vert.glsl");
            writer.write("uniform mat4 transform;\r\n            uniform mat4 modelviewMatrix;\r\n            uniform mat4 modelMatrix;\r\n            uniform mat3 normalMatrix;\r\n            uniform vec3 lightNormal;\r\n            uniform mat4 modelViewInv; \r\n            \r\n            \r\n            attribute vec4 position;\r\n            attribute vec4 color;\r\n            attribute vec3 normal;\r\n            \r\n            varying vec4 vertColor;\r\n            varying vec3 vertNormal;\r\n            varying vec3 vertLightDir;\r\n            varying vec3 vertWorldNormal;\r\n            varying vec4 vertWorldPos; \r\n            varying vec4 posN;\r\n            \r\n            void main() {\r\n              gl_Position = transform * position;\r\n              vec4 f_normal = vec4(normal.x, normal.y, normal.z, 1.0); // vec4(normalize(normalMatrix * normal), 1.0);//\r\n              f_normal = position * modelViewInv;\r\n              //posN =  \r\n              vertColor = color;\r\n              vertNormal = f_normal.xyz;\r\n              vertLightDir = normal;\r\n              vertWorldNormal  = normalMatrix * normal;\r\n              vertWorldPos = transform * position;\r\n            }");
            writer.close();
            writer = new FileWriter("kusudama.glsl");
            writer.write("#ifdef GL_ES\r\nprecision mediump float;\r\nprecision mediump int;\r\n#endif\r\n\r\nvarying vec4 vertColor;\r\n\r\n//Model space normal direction of the current fragment\r\n//since we're on a sphere, this is literally just the fragment's position in \r\n//modelspace\r\nvarying vec3 vertNormal;\r\n\r\n//This shader can display up to 30 cones (represented by 30 4d vectors) \r\n// alphachannel represents radius, rgb channels represent xyz coordinates of \r\n// the cone direction vector in model space\r\nuniform vec4 coneSequence[30];\r\nuniform int coneCount; \r\n \r\n//Make this \"true\" for sceendoor transparency (randomly discarding fragments)\r\n//so that you can blur the result in another pass. Otherwise make it  \r\n//false for a solid shell.  \r\nuniform bool multiPass;\r\n\r\n//Following three varyings are \r\n//Only used for fake lighting. \r\n//Not conceptually relevant\r\nvarying vec3 vertWorldNormal;\r\nvarying vec3 vertLightDir;\r\nvarying vec4 vertWorldPos;\r\n\r\n\r\n///NOISE FUNCTIONS FOR FANCY TRANSPARENCY RENDERING\r\nfloat hash( uint n ) { // from https://www.shadertoy.com/view/llGSzw  Base: Hugo Elias. ToFloat: http://iquilezles.org/www/articles/sfrand/sfrand.htm\r\n\tn = (n << 13U) ^ n;\r\n    n = n * (n * n * 15731U + 789221U) + 1376312589U;\r\n    return uintBitsToFloat( (n>>9U) | 0x3f800000U ) - 1.;\r\n}\r\n\r\nfloat noise(vec2 U) {\r\n    return hash(uint(U.x+5000.0*U.y));\r\n}\r\n\r\nbool randBit(vec2 U) {\r\n\tfloat dist2 = 1.0;\r\n\treturn 0.5 < (noise(U) * 4. -(noise(U+vec2(dist2,0.))+noise(U+vec2(0.,dist2))+noise(U-vec2(0.,dist2))+noise(U-vec2(dist2,0.))) + 0.5);\r\n}\r\n///END OF NOISE FUNCTIONS FOR FANCY TRANSPARENCY RENDERING.\r\n\r\nbool isInInterConePath(in vec3 normalDir, in vec4 tangent1, in vec4 cone1, in vec4 tangent2, in vec4 cone2) {\t\t\t\r\n\tvec3 c1xc2 = cross(cone1.xyz, cone2.xyz);\t\t\r\n\tfloat c1c2dir = dot(normalDir, c1xc2);\r\n\t\t\r\n\tif(c1c2dir < 0.0) { \r\n\t\tvec3 c1xt1 = cross(cone1.xyz, tangent1.xyz); \r\n\t\tvec3 t1xc2 = cross(tangent1.xyz, cone2.xyz);\t\r\n\t\tfloat c1t1dir = dot(normalDir, c1xt1);\r\n\t\tfloat t1c2dir = dot(normalDir, t1xc2);\r\n\t\t\r\n\t \treturn (c1t1dir > 0.0 && t1c2dir > 0.0); \r\n\t\t\t\r\n\t}else {\r\n\t\tvec3 t2xc1 = cross(tangent2.xyz, cone1.xyz);\t\r\n\t\tvec3 c2xt2 = cross(cone2.xyz, tangent2.xyz);\t\r\n\t\tfloat t2c1dir = dot(normalDir, t2xc1);\r\n\t\tfloat c2t2dir = dot(normalDir, c2xt2);\r\n\t\t\r\n\t\treturn (c2t2dir > 0.0 && t2c1dir > 0.0);\r\n\t}\t\r\n\treturn false;\r\n}\r\n\r\n//determines the current draw condition based on the desired draw condition in the setToArgument\r\n// -3 = disallowed entirely; \r\n// -2 = disallowed and on tangentCone boundary\r\n// -1 = disallowed and on controlCone boundary\r\n// 0 =  allowed and empty; \r\n// 1 =  allowed and on controlCone boundary\r\n// 2  = allowed and on tangentCone boundary\r\nint getAllowabilityCondition(in int currentCondition, in int setTo) {\r\n\tif((currentCondition == -1 || currentCondition == -2)\r\n\t\t&& setTo >= 0) {\r\n\t\treturn currentCondition *= -1;\r\n\t} else if(currentCondition == 0 && (setTo == -1 || setTo == -2)) {\r\n\t\treturn setTo *=-2;\r\n\t}  \t\r\n\treturn max(currentCondition, setTo);\r\n}\r\n\r\n\r\n\r\n//returns 1 if normalDir is beyond (cone.a) radians from cone.rgb\r\n//returns 0 if normalDir is within (cone.a + boundaryWidth) radians from cone.rgb \r\n//return -1 if normalDir is less than (cone.a) radians from cone.rgb\r\nint isInCone(in vec3 normalDir, in vec4 cone, in float boundaryWidth) {\r\n\tfloat arcDistToCone = acos(dot(normalDir, cone.rgb));\r\n\tif(arcDistToCone > (cone.a+(boundaryWidth/2.))) {\r\n\t\treturn 1; \r\n\t}\r\n\tif(arcDistToCone < cone.a-(boundaryWidth/2.)) {\r\n\t\treturn -1;\r\n\t}\r\n\treturn 0;\r\n} \r\n\r\n//returns a color corresponding to the allowability of this region, or otherwise the boundaries corresponding \r\n//to various cones and tangentCone \r\nvec4 colorAllowed(in vec3 normalDir,  in int coneCount, in float boundaryWidth) {\r\n\tnormalDir = normalize(normalDir);\r\n\tint currentCondition = -3;\r\n\t\r\n\tif(coneCount == 1) {\r\n\t\tvec4 cone = coneSequence[0];\r\n\t\tint inCone = isInCone(normalDir, cone, boundaryWidth);\r\n\t\tinCone = inCone == 0 ? -1 : inCone < 0 ? 0 : -3;\r\n\t\tcurrentCondition = getAllowabilityCondition(currentCondition, inCone);\r\n\t} else {\r\n\t\tfor(int i=0; i<coneCount-1; i+=3) {\r\n\t\t\t\r\n\t\t\tint idx = i*3; \r\n\t\t\tvec4 cone1 = coneSequence[idx];\r\n\t\t\tvec4 tangent1 = coneSequence[idx+1];\t\t\t\r\n\t\t\tvec4 tangent2 = coneSequence[idx+2];\t\t\t\r\n\t\t\tvec4 cone2 = coneSequence[idx+3];\r\n\t\t\t\t\t\t\t\t\t\t\r\n\t\t\tint inCone1 = isInCone(normalDir, cone1, boundaryWidth);\r\n\t\t\t\r\n\t\t\tinCone1 = inCone1 == 0 ? -1 : inCone1 < 0 ? 0 : -3;\r\n\t\t\tcurrentCondition = getAllowabilityCondition(currentCondition, inCone1);\r\n\t\t\t\t\r\n\t\t\tint inCone2 = isInCone(normalDir, cone2, boundaryWidth);\r\n\t\t\tinCone2 =  inCone2 == 0 ? -1 : inCone2  < 0 ? 0 : -3;\r\n\t\t\tcurrentCondition = getAllowabilityCondition(currentCondition, inCone2);\r\n\t\t\r\n\t\t\tint inTan1 = isInCone(normalDir, tangent1, boundaryWidth); \r\n\t\t\tint inTan2 = isInCone(normalDir, tangent2, boundaryWidth);\r\n\t\t\t\r\n\t\t\tif( inTan1 < 1. || inTan2  < 1.) {\t\t\t\r\n\t\t\t\tinTan1 =  inTan1 == 0 ? -2 : -3;\r\n\t\t\t\tcurrentCondition = getAllowabilityCondition(currentCondition, inTan1);\r\n\t\t\t\tinTan2 =  inTan2 == 0 ? -2 : -3;\r\n\t\t\t\tcurrentCondition = getAllowabilityCondition(currentCondition, inTan2);\r\n\t\t\t} else {\t\t\t\t \r\n\t\t\t\tbool inIntercone = isInInterConePath(normalDir, tangent1, cone1, tangent2, cone2);\r\n\t\t\t\tint interconeCondition = inIntercone ? 0 : -3; \r\n\t\t\t\tcurrentCondition = getAllowabilityCondition(currentCondition, interconeCondition);\t\t\t\t\t\r\n\t\t\t}\r\n\t\t}\r\n\t}\t\r\n\t\r\n\tvec4 result = vertColor;\r\n\t\r\n\tif(multiPass && (currentCondition == -3 || currentCondition > 0)) {\r\n\t\t\r\n\t\t/////////\r\n\t\t//CODE FOR FANCY BLURRED TRANSPARENCY. \r\n\t\t//NOT OTHERWISE CONCEPTUALLY RELEVANT TO \r\n\t\t//TO VISUALIZATION\r\n\t\t////////\r\n\t\t\r\n\t\tvec3 randDir = vec3(normalDir.x  * noise(normalDir.xy)/50.0,  normalDir.y  * noise(normalDir.yz)/50.0, normalDir.z  * noise(normalDir.zx)/50.0);\r\n\t\trandDir = normalDir;\r\n\t\tfloat zAdd = abs(vertWorldPos.z);\r\n\t\tfloat lon = atan(randDir.x/randDir.z) + 3.14159265/2.0;\r\n\t\tfloat lat = atan(randDir.y/randDir.x) + 3.14159265/2.0;\r\n\t\t\t\t\r\n\t\tbool latDraw = randBit(vec2(lat, lon));//mod(lat, 0.005) < 0.00499;\r\n\t\tbool lonDraw = randBit(vec2(lon, lat));//mod(lon, 0.005) < 0.00499;\r\n\t\t\t\r\n\t\tif(randBit(vec2(lon, lat))) {\t\t\r\n\t\t\tresult = vec4(0.0,0.0,0.0,0.0);\t\r\n\t\t}\r\n\t\t////////\r\n\t\t//END CODE FOR FANCY BLURRED TRANSPARENCY\r\n\t\t///////\r\n\t} else if (currentCondition != 0) {\r\n\t\r\n\t\tfloat onTanBoundary = abs(currentCondition) == 2 ? 0.3 : 0.0; \r\n\t\tfloat onConeBoundary = abs(currentCondition) == 1 ? 0.3 : 0.0;\t\r\n\t\r\n\t\t//return distCol;\r\n\t\tresult += vec4(0.0, onConeBoundary, onTanBoundary, 1.0);\r\n\t} else {\r\n\t\tdiscard;\r\n\t}\r\n\treturn result;\r\n\t\t\t\r\n}\r\n\r\nvoid main() {\r\n\r\n  vec3 normalDir = normalize(vertNormal); // the vertex normal in Model Space.\r\n  float lightScalar = dot(vertLightDir, vec3(0.5,-1.,0.5)); \r\n  lightScalar *= lightScalar*lightScalar;\r\n  vec4 colorAllowed = colorAllowed(normalDir, coneCount, 0.02);  \r\n\r\n  if(colorAllowed.a == 0.0)\r\n  \tdiscard;\r\n  \t\r\n  colorAllowed += (colorAllowed + fwidth(colorAllowed)); \r\n  colorAllowed /= 2.0;\r\n  vec3 lightCol = vec3(1.0,0.8,0.0);\r\n  float gain = vertWorldNormal.z < 0 ? -0.3 : 0.5;\r\n colorAllowed.rgb = (colorAllowed.rgb + lightCol*(lightScalar + gain)) / 2.;\r\n vec4 specCol = vec4(1.0, 1.0, 0.6, colorAllowed.a);  \r\n colorAllowed = colorAllowed.g > 0.8 ? colorAllowed+specCol : colorAllowed;  \t\r\n  \t\r\n  gl_FragColor = colorAllowed;\r\n}");
            writer.close();
            writer = new FileWriter("kusudama_stencil.glsl");
            writer.write("#ifdef GL_ES\r\nprecision mediump float;\r\nprecision mediump int;\r\n#endif\r\n\r\nuniform float fraction;\r\n\r\nvarying vec4 vertColor;\r\nvarying vec3 vertNormal;\r\nvarying vec3 vertWorldNormal;\r\nvarying vec4 posN;\r\nvarying vec3 vertLightDir;\r\nvarying vec4 vertWorldPos; \r\n\r\nuniform vec4 coneSequence[30]; //this shader can display up to 10 cones (represented by 30 4d vectors)  \r\nuniform int coneCount; \r\n\r\n\r\nbool isInInterConePath(in vec3 normalDir, in vec4 tangent1, in vec4 cone1, in vec4 tangent2, in vec4 cone2) {\t\t\t\r\n\tvec3 c1xc2 = cross(cone1.xyz, cone2.xyz);\t\t\r\n\tfloat c1c2dir = dot(normalDir, c1xc2);\r\n\t\t\r\n\tif(c1c2dir < 0.0) { \r\n\t\tvec3 c1xt1 = cross(cone1.xyz, tangent1.xyz); \r\n\t\tvec3 t1xc2 = cross(tangent1.xyz, cone2.xyz);\t\r\n\t\tfloat c1t1dir = dot(normalDir, c1xt1);\r\n\t\tfloat t1c2dir = dot(normalDir, t1xc2);\r\n\t\t\r\n\t \treturn (c1t1dir > 0.0 && t1c2dir > 0.0); \r\n\t\t\t\r\n\t}else {\r\n\t\tvec3 t2xc1 = cross(tangent2.xyz, cone1.xyz);\t\r\n\t\tvec3 c2xt2 = cross(cone2.xyz, tangent2.xyz);\t\r\n\t\tfloat t2c1dir = dot(normalDir, t2xc1);\r\n\t\tfloat c2t2dir = dot(normalDir, c2xt2);\r\n\t\t\r\n\t\treturn (c2t2dir > 0.0 && t2c1dir > 0.0);\r\n\t}\t\r\n\treturn false;\r\n}\r\n\r\nfloat rand(vec2 co){\r\n    return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);\r\n}\r\n\r\nvec4 drawInterConePathDisparity(in vec3 normalDir, in vec4 tangent1, in vec4 cone1, in vec4 tangent2, in vec4 cone2) {\t\t\r\n\tvec3 c1xc2 = cross(cone1.xyz, cone2.xyz);\t\t\r\n\tfloat c1c2dir = dot(normalDir, c1xc2);\r\n\tvec4 result = vec4(0.0,0.0,0.0,1.0);\t\r\n\t\r\n\tif(c1c2dir < 0.0) { \r\n\t\tvec3 c1xt1 = cross(cone1.xyz, tangent1.xyz); \r\n\t\tvec3 t1xc2 = cross(tangent1.xyz, cone2.xyz);\t\r\n\t\tfloat c1t1dir = dot(normalDir, c1xt1);\r\n\t\tfloat t1c2dir = dot(normalDir, t1xc2);\r\n\t\t\r\n\t \tif(c1t1dir > 0.0 && t1c2dir > 0.0) { \r\n\t\t\tresult = vec4(0.5, 0.5, 0.0, 1.0);\r\n\t\t}\r\n\t}else {\r\n\t\tvec3 t2xc1 = cross(tangent2.xyz, cone1.xyz);\t\r\n\t\tvec3 c2xt2 = cross(cone2.xyz, tangent2.xyz);\t\r\n\t\tfloat t2c1dir = dot(normalDir, t2xc1);\r\n\t\tfloat c2t2dir = dot(normalDir, c2xt2);\r\n\t\t\r\n\t\tif(c2t2dir > 0.0 && t2c1dir > 0.0) { \r\n\t\t\tresult  = vec4(0.0, 0.5, 0.5, 1.0);\r\n\t\t}\r\n\t}\t\r\n\treturn result;\r\n}\r\n\r\n//determines the current draw condition based on the desired draw condition in the setToArgument\r\n// -3 = disallowed entirely; \r\n// -2 = disallowed and on tangentCone boundary\r\n// -1 = disallowed and on controlCone boundary\r\n// 0 =  allowed and empty; \r\n// 1 =  allowed and on controlCone boundary\r\n// 2  = allowed and on tangentCone boundary\r\nint getAllowabilityCondition(in int currentCondition, in int setTo) {\r\n\tif((currentCondition == -1 || currentCondition == -2)\r\n\t\t&& setTo >= 0) {\r\n\t\treturn currentCondition *= -1;\r\n\t} else if(currentCondition == 0 && (setTo == -1 || setTo == -2)) {\r\n\t\treturn setTo *=-2;\r\n\t}  \t\r\n\treturn max(currentCondition, setTo);\r\n}\r\n\r\n\r\n\r\n//returns 1 if normalDir is beyond (cone.a) radians from cone.rgb\r\n//returns 0 if normalDir is within (cone.a + boundaryWidth) radians from cone.rgb \r\n//return -1 if normalDir is less than (cone.a) radians from cone.rgb\r\nint isInCone(in vec3 normalDir, in vec4 cone, in float boundaryWidth) {\r\n\tfloat arcDistToCone = acos(dot(normalDir, cone.rgb));\r\n\tif(arcDistToCone > (cone.a+(boundaryWidth/2.))) {\r\n\t\treturn 1; \r\n\t}\r\n\tif(arcDistToCone < cone.a-(boundaryWidth/2.)) {\r\n\t\treturn -1;\r\n\t}\r\n\treturn 0;\r\n} \r\n\r\n//returns a color corresponding to the allowability of this region, or otherwise the boundaries corresponding \r\n//to various cones and tangentCone \r\nvec4 colorAllowed(in vec3 normalDir,  in int coneCount, in float boundaryWidth) {\r\n\tnormalDir = normalize(normalDir);\r\n\tint currentCondition = -3;\r\n\tvec4 distCol = vec4(0.0, 0.0, 0.0, 1.0);\r\n\tif(coneCount == 1) {\r\n\t\t//ivec2 coneIDX = ivec2(1, 0);\r\n\t\tvec4 cone = coneSequence[0];//texelFetch(coneSequence, coneIDX, 0);\r\n\t\tint inCone = isInCone(normalDir, cone, boundaryWidth);\r\n\t\tinCone = inCone == 0 ? -1 : inCone < 0 ? 0 : -3;\r\n\t\tcurrentCondition = getAllowabilityCondition(currentCondition, inCone);\r\n\t\t\t\r\n\t} else {\r\n\t\tfor(int i=0; i<coneCount-1; i+=3) {\r\n\t\t\t\r\n\t\t\tint idx = i*3; \r\n\t\t\tvec4 cone1 = coneSequence[idx];\r\n\t\t\tvec4 tangent1 = coneSequence[idx+1];\t\t\t\r\n\t\t\tvec4 tangent2 = coneSequence[idx+2];\t\t\t\r\n\t\t\tvec4 cone2 = coneSequence[idx+3];\r\n\t\t\t\t\t\t\r\n\t\t\t\t\r\n\t\t\tint inCone1 = isInCone(normalDir, cone1, boundaryWidth);\r\n\t\t\t\r\n\t\t\tinCone1 = inCone1 == 0 ? -1 : inCone1 < 0 ? 0 : -3;\r\n\t\t\tcurrentCondition = getAllowabilityCondition(currentCondition, inCone1);\r\n\t\t\t\t\r\n\t\t\tint inCone2 = isInCone(normalDir, cone2, boundaryWidth);\r\n\t\t\tinCone2 =  inCone2 == 0 ? -1 : inCone2  < 0 ? 0 : -3;\r\n\t\t\tcurrentCondition = getAllowabilityCondition(currentCondition, inCone2);\r\n\t\t\r\n\t\t\tint inTan1 = isInCone(normalDir, tangent1, boundaryWidth); \r\n\t\t\tint inTan2 = isInCone(normalDir, tangent2, boundaryWidth);\r\n\t\t\t\r\n\t\t\tif( inTan1 < 1. || inTan2  < 1.) {\t\t\t\r\n\t\t\t\tinTan1 =  inTan1 == 0 ? -2 : -3;\r\n\t\t\t\tcurrentCondition = getAllowabilityCondition(currentCondition, inTan1);\r\n\t\t\t\tinTan2 =  inTan2 == 0 ? -2 : -3;\r\n\t\t\t\tcurrentCondition = getAllowabilityCondition(currentCondition, inTan2);\r\n\t\t\t} else {\t\t\t\t \r\n\t\t\t\tbool visualize = false;\r\n\t\t\t\tif( ! visualize ) {\r\n\t\t\t\t\tbool inIntercone = isInInterConePath(normalDir, tangent1, cone1, tangent2, cone2);\r\n\t\t\t\t\tint interconeCondition = inIntercone ? 0 : -3;// && (inTangent1 > 0 || inTangent2 > 0) ? -3 : 0; \r\n\t\t\t\t\tcurrentCondition = getAllowabilityCondition(currentCondition, interconeCondition);\t\t\t\t\t\r\n\t\t\t\t} else { \r\n\t\t\t\t\t//visualization code, set visualize = true to get a sense of how the math works.\t\r\n\t\t\t\t\tvec4 intervec = drawInterConePathDisparity(normalDir, tangent1, cone1, tangent2, cone2);\r\n\t\t\t\t\tif(inCone1 == 0 || inCone2 == 0) intervec.rgb += .3;\t\t\t\t\r\n\t\t\t\t\treturn intervec;\r\n\t\t\t\t}\r\n\t\t\t}\r\n\t\t}\r\n\t}\t\r\n\t\r\n\tvec4 result = vertColor;\r\n\t\r\n\tif(currentCondition == -3) {\r\n\t\treturn result; \r\n\t} else { \r\n\t\tdiscard;\r\n\t}\r\n\treturn result;\r\n\t\t\r\n\t\r\n}\r\n\r\nvoid main() {\r\n\r\n  vec3 normalDir = normalize(vertNormal);\r\n  vec4 colorAllowed = colorAllowed(normalDir, coneCount, 0.02);   \t\r\n  if(vertWorldNormal.z < 0.0) {\r\n  \tcolorAllowed.rgb /= 2.0;\r\n  }\r\n  vec4 scvertWorldPos = vertWorldPos; \r\n  scvertWorldPos.z += 0.5; \r\n  scvertWorldPos.z /= -2.0;\r\n  gl_FragColor = colorAllowed;//vec4(scvertWorldPos.z, scvertWorldPos.z, scvertWorldPos.z, 1.0); ;\r\n}");
            writer.close();
        } catch (IOException e) {
            System.out.println("Can't write file.");
            e.printStackTrace();
        }
        pa = p;
        currentDrawSurface = pa.g;
        stencil = pa.createGraphics(p.width, p.height, PConstants.P3D);
        display = pa.createGraphics(p.width, p.height, PConstants.P3D);
        stencil.noSmooth();
        display.smooth(8);
        System.out.println(p.sketchPath());
        blurshader = pa.loadShader("blur-sep.glsl");
        blurshader.set("blurSize", 20);
        blurshader.set("sigma", 9f);
        Kusudama.kusudamaShader = pa.loadShader("kusudama.glsl",
                "kusudama_vert.glsl");
        Kusudama.kusudamaStencil = pa.loadShader("kusudama_stencil.glsl",
                "kusudama_vert.glsl");

    }

    public void drawInstructions(PGraphics pg, String append) {
        String appended = append == null ? "" : "-" + append;

        String instructionText = "-Click and drag to move the selected pin.\n"
                + "-To select a different pin, use the Up and Down arrows.\n"
                + "-Use the mouse wheel to rotate the pin about its (red) Y axis.\n"
                + "-Hold shift while using the mouse wheel to rotate the pin about its (blue) Z axis.\n"
                + "-Hold ctrl while using the mouse wheel to rotate the pin about its (green) X axis. \n"
                + appended;
        // pg.textMode(pa.MODEL);
        if (pg.textFont == null)
            pg.textSize(12);
        else {
            pg.textSize(12);
        }
        // PFont instructionFont = pa.createFont(pg.textFont.getName(), 42);
        pg.fill(0, 0, 0, 90);
        float boxW = pg.textWidth(instructionText);
        float boxH = (pg.textAscent() + pg.textDescent()) * (instructionText.split("\n").length);
        pg.rect((-pa.width / 2f) + 30, (-pa.height / 2f) + 15, boxW + 25, boxH + 40);
        pg.fill(255, 255, 255, 255);
        pg.emissive(255, 255, 255);
        pg.text(instructionText, (-pa.width / 2f) + 50f, -pa.height / 2f + 40f);

    }

    public void line(PGraphics pg, PVector p1, PVector p2) {
        pg.line(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
    }

    public void point(PGraphics pg, PVector p) {
        pg.point(p.x, p.y, p.z);
    }

    public void drawPins(PGraphics pg, IKPin activePin,
                         float zoomScalar, float drawSize,
                         boolean cubeMode, Axes cubeAxes) {

        if (activePin != null) {
            Axes ellipseAx;
            ellipseAx = cubeMode ? cubeAxes : activePin.getAxes();
            PVector pinLoc = screenOf(pg, ellipseAx.origin(), zoomScalar);
            PVector pinX = screenOf(pg, Axes.toPVector(ellipseAx.x_().getScaledTo(drawSize)), zoomScalar);
            PVector pinY = screenOf(pg, Axes.toPVector(ellipseAx.y_().getScaledTo(drawSize)), zoomScalar);
            PVector pinZ = screenOf(pg, Axes.toPVector(ellipseAx.z_().getScaledTo(drawSize)), zoomScalar);
            pg.fill(255, 255, 255, 150);
            pg.stroke(255, 0, 255);
            float totalPriorities = activePin.getXPriority() + activePin.getYPriority()
                    + activePin.getZPriority();
            pg.ellipse(pinLoc.x, pinLoc.y, zoomScalar * 50, zoomScalar * 50);

            PVector effectorO = screenOf(pg, Axes.toPVector(activePin.forBone().localAxes().origin_()), zoomScalar);
            PVector effectorX = screenOf(pg, Axes.toPVector(activePin.forBone().localAxes().x_().getScaledTo(drawSize)),
                    zoomScalar);
            PVector effectorY = screenOf(pg, Axes.toPVector(activePin.forBone().localAxes().y_().getScaledTo(drawSize)),
                    zoomScalar);
            PVector effectorZ = screenOf(pg, Axes.toPVector(activePin.forBone().localAxes().z_().getScaledTo(drawSize)),
                    zoomScalar);
            pg.stroke(255, 255, 255, 150);

            if (!cubeMode) {
                float xPriority = activePin.getXPriority();
                float yPriority = activePin.getYPriority();
                float zPriority = activePin.getZPriority();
                drawPinEffectorHints(
                        pg,
                        pinLoc,
                        pinX, pinY, pinZ,
                        effectorO,
                        effectorX, effectorY, effectorZ,
                        xPriority, yPriority, zPriority, totalPriorities);
            }
        }
    }

    public void drawPinEffectorHints(PGraphics pg,
                                     PVector pinLoc,
                                     PVector pinX, PVector pinY, PVector pinZ,
                                     PVector effectorO,
                                     PVector effectorX, PVector effectorY, PVector effectorZ,
                                     float xPriority, float yPriority, float zPriority, float totalpriorities) {

        pg.line(pinLoc.x, pinLoc.y, pinLoc.z, effectorO.x, effectorO.y, effectorO.z);
        pg.stroke(0, 255, 0, 150);
        pg.strokeWeight(2f * xPriority / totalpriorities);
        pg.line(pinX.x, pinX.y, pinX.z, effectorX.x, effectorX.y, effectorX.z);
        pg.stroke(255, 0, 0, 150);
        pg.strokeWeight(2f * yPriority / totalpriorities);
        pg.line(pinY.x, pinY.y, pinY.z, effectorY.x, effectorY.y, effectorY.z);
        pg.stroke(0, 0, 255, 150);
        pg.strokeWeight(2f * zPriority / totalpriorities);
        pg.line(pinZ.x, pinZ.y, pinZ.z, effectorZ.x, effectorZ.y, effectorZ.z);

    }

    public void drawPass(int mode, float drawSize, Runnable preArmatureDraw, PGraphics buffer, Armature armature) {
        Kusudama.renderMode = mode;
        Bone.renderMode = mode;
        Axes.renderMode = mode;
        if (preArmatureDraw != null)
            preArmatureDraw.run();
        armature.drawMe(buffer, 100, drawSize);
    }

    public PVector screenOf(PGraphics pg, PVector pt, float zoomScalar) {
        return new PVector(
                (pg.screenX(pt.x, pt.y, pt.z) * zoomScalar) - orthoWidth / 2f,
                (pg.screenY(pt.x, pt.y, pt.z) * zoomScalar) - orthoHeight / 2f);
    }

    private PGraphics currentDrawSurface;

    public void drawScene(float zoomScalar, float drawSize,
                          Runnable additionalDraw,
                          Armature armature,
                          String usageInstructions,
                          IKPin activePin, Axes cubeAxes, boolean cubeEnabled) {
        Kusudama.enableMultiPass(true);
        currentDrawSurface = stencil;

        stencil.beginDraw();
        setCamera(stencil, zoomScalar);
        drawPass(0, drawSize, additionalDraw, stencil, armature);
        stencil.endDraw();

        currentDrawSurface = display;
        display.beginDraw();
        setSceneAndCamera(display, zoomScalar);
        drawPass(1, drawSize, additionalDraw, display, armature);
        blurshader.set("mask", stencil);
        blurshader.set("horizontalPass", 0);
        display.filter(blurshader);
        blurshader.set("horizontalPass", 1);
        display.filter(blurshader);
        display.endDraw();

        currentDrawSurface = pa.g;
        setCamera(pa.g, zoomScalar);
        pa.background(80, 150, 190);
        pa.imageMode(PConstants.CENTER);
        pa.image(display, 0, 0, orthoWidth, orthoHeight);
        pa.resetMatrix();
        drawPins(pa.g, activePin, zoomScalar, drawSize, cubeEnabled, cubeAxes);
        pa.resetMatrix();
        float cx = pa.width;
        float cy = pa.height;
        pa.ortho(-cx / 2f, cx / 2f, -cy / 2f, cy / 2f, -1000, 1000);
        drawInstructions(pa.g, usageInstructions);
        drawPins(pa.g, activePin, drawSize, zoomScalar, cubeEnabled, cubeAxes);
        drawInstructions(pa.g, usageInstructions);

    }

    PVector mouse = new PVector(0, 0, 0);
    PVector cameraPosition = new PVector(0, 0, 70);
    PVector lookAt = new PVector(0, 0, 0);
    PVector up = new PVector(0, 1, 0);

    public void camera(PVector cp, PVector so, PVector up, PGraphics pg) {
        pg.camera(cp.x, cp.y, cp.z, so.x, so.y, so.z, up.x, up.y, up.z);
    }

    public void setSceneAndCamera(PGraphics pg, float zoomScalar) {
        setCamera(pg, zoomScalar);
        pg.directionalLight(148, 148, 148, 0, 100, 100);
        pg.directionalLight(148, 148, 148, 0, 100, -100);
        pg.directionalLight(148, 148, 148, 100, 100, 0);
        pg.directionalLight(148, 148, 148, -100, 100, 0);
        pg.directionalLight(48, 48, 48, 100, -10, 100);
        pg.directionalLight(48, 48, 48, 100, -10, -100);
        pg.directionalLight(48, 48, 48, -100, -10, 100);
        pg.directionalLight(48, 48, 48, -100, -10, -100);

    }

    float orthoHeight, orthoWidth;

    public void setCamera(PGraphics pg, float zoomScalar) {
        pg.clear();
        orthoHeight = pa.height * zoomScalar;
        orthoWidth = ((float) pa.width / (float) pa.height) * orthoHeight;
        mouse.x = (pa.mouseX - (pa.width / 2f)) * (orthoWidth / pa.width);
        mouse.y = (pa.mouseY - (pa.height / 2f)) * (orthoHeight / pa.height);
        camera(cameraPosition, lookAt, up, pg);
        pg.ortho(-orthoWidth / 2f, orthoWidth / 2f, -orthoHeight / 2f, orthoHeight / 2f, -1000, 1000);
    }

    /**
     * @return the draw surface this class is currently operating on.
     * This is used as kind of hack, so I don't have to bother writing
     * interfaces just to render a box when using multipass.
     */
    public PGraphics getCurrentDrawSurface() {
        return currentDrawSurface;
    }

}
