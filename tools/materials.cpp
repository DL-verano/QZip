#include "materials.h"
#include "include_opengl.h"

void gl_set_material(Material m) {
    float ambient[] = {0.0f, 0.0f, 0.0f, 1.0f};
    float diffuse[] = {0.0f, 0.0f, 0.0f, 1.0f};
    float specular[] = {0.0f, 0.0f, 0.0f, 1.0f};
    float emission[] = {0.3f, 0.3f, 0.3f, 1.0f};
    float shininess[] = {0.0f};
    switch (m) {
    case Material::Silver:
        // Ambient
        ambient[0] = 0.19225f;
        ambient[1] = 0.19225f;
        ambient[2] = 0.19225f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.50754f;
        diffuse[1] = 0.50754f;
        diffuse[2] = 0.50754f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.508273f;
        specular[1] = 0.508273f;
        specular[2] = 0.508273f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 51.2f;
        break;
    case Material::Gold:
        // Ambient
        ambient[0] = 0.24725f;
        ambient[1] = 0.1995f;
        ambient[2] = 0.0745f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.75164f;
        diffuse[1] = 0.60648f;
        diffuse[2] = 0.22648f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.928281f;
        specular[1] = 0.855802f;
        specular[2] = 0.666065f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 51.2f;
        break;
    case Material::Jade:
        // Ambient
        ambient[0] = 0.135f;
        ambient[1] = 0.2225f;
        ambient[2] = 0.1575f;
        ambient[3] = 0.95f;
        // Diffuse
        diffuse[0] = 0.54f;
        diffuse[1] = 0.89f;
        diffuse[2] = 0.63f;
        diffuse[3] = 0.95f;
        // Specular
        specular[0] = 0.316228f;
        specular[1] = 0.316228f;
        specular[2] = 0.316228f;
        specular[3] = 0.95f;
        // Shininess
        shininess[0] = 12.8f;
        break;
    case Material::LightBlue:
        // Ambient
        ambient[0] = 0.0f;
        ambient[1] = 0.5f;
        ambient[2] = 0.75f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.0f;
        diffuse[1] = 0.5f;
        diffuse[2] = 1.0f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.75f;
        specular[1] = 0.75f;
        specular[2] = 0.75f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 64.0f;
        break;
    case Material::Emerald:
        // Ambient
        ambient[0] = 0.0215f;
        ambient[1] = 0.1745f;
        ambient[2] = 0.0215f;
        ambient[3] = 0.55f;
        // Diffuse
        diffuse[0] = 0.07568f;
        diffuse[1] = 0.61424f;
        diffuse[2] = 0.07568f;
        diffuse[3] = 0.55f;
        // Specular
        specular[0] = 0.633f;
        specular[1] = 0.727811f;
        specular[2] = 0.633f;
        specular[3] = 0.55f;
        // Shininess
        shininess[0] = 76.8f;
        break;
    case Material::PolishedSilver:
        // Ambient
        ambient[0] = 0.23125f;
        ambient[1] = 0.23125f;
        ambient[2] = 0.23125f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.2775f;
        diffuse[1] = 0.2775f;
        diffuse[2] = 0.2775f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.773911f;
        specular[1] = 0.773911f;
        specular[2] = 0.773911f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 89.6f;
        break;
    case Material::Chrome:
        // Ambient
        ambient[0] = 0.25f;
        ambient[1] = 0.25f;
        ambient[2] = 0.25f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.4f;
        diffuse[1] = 0.4f;
        diffuse[2] = 0.4f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.774597f;
        specular[1] = 0.774597f;
        specular[2] = 0.774597f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 76.8f;
        break;
    case Material::Copper:
        // Ambient
        ambient[0] = 0.19125f;
        ambient[1] = 0.0735f;
        ambient[2] = 0.0225f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.7038f;
        diffuse[1] = 0.27048f;
        diffuse[2] = 0.0828f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.256777f;
        specular[1] = 0.137622f;
        specular[2] = 0.086014f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 12.8f;
        break;
    case Material::PolishedGold:
        // Ambient
        ambient[0] = 0.24725f;
        ambient[1] = 0.2245f;
        ambient[2] = 0.0645f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.34615f;
        diffuse[1] = 0.3143f;
        diffuse[2] = 0.0903f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.797357f;
        specular[1] = 0.723991f;
        specular[2] = 0.208006f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 83.2f;
        break;
    case Material::Pewter:
        // Ambient
        ambient[0] = 0.105882f;
        ambient[1] = 0.058824f;
        ambient[2] = 0.113725f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.427451f;
        diffuse[1] = 0.470588f;
        diffuse[2] = 0.541176f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.333333f;
        specular[1] = 0.333333f;
        specular[2] = 0.521569f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 9.84615f;
        break;
    case Material::Obsidian:
        // Ambient
        ambient[0] = 0.05375f;
        ambient[1] = 0.05f;
        ambient[2] = 0.06625f;
        ambient[3] = 0.82f;
        // Diffuse
        diffuse[0] = 0.18275f;
        diffuse[1] = 0.17f;
        diffuse[2] = 0.22525f;
        diffuse[3] = 0.82f;
        // Specular
        specular[0] = 0.332741f;
        specular[1] = 0.328634f;
        specular[2] = 0.346435f;
        specular[3] = 0.82f;
        // Shininess
        shininess[0] = 38.4f;
        break;
    case Material::BlackPlastic:
        // Ambient
        ambient[0] = 0.0f;
        ambient[1] = 0.0f;
        ambient[2] = 0.0f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.01f;
        diffuse[1] = 0.01f;
        diffuse[2] = 0.01f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.5f;
        specular[1] = 0.5f;
        specular[2] = 0.5f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 32.0f;
        break;
    case Material::PolishedBronze:
        // Ambient
        ambient[0] = 0.25f;
        ambient[1] = 0.148f;
        ambient[2] = 0.006475f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.4f;
        diffuse[1] = 0.2368f;
        diffuse[2] = 0.1036f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.774597f;
        specular[1] = 0.458561f;
        specular[2] = 0.200621f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 76.8f;
        break;
    case Material::PolishedCopper:
        // Ambient
        ambient[0] = 0.2295f;
        ambient[1] = 0.08825f;
        ambient[2] = 0.0275f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.5508f;
        diffuse[1] = 0.2118f;
        diffuse[2] = 0.066f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.580594f;
        specular[1] = 0.223257f;
        specular[2] = 0.0695701f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 51.2f;
        break;
    case Material::Pearl:
        // Ambient
        ambient[0] = 0.25f;
        ambient[1] = 0.20725f;
        ambient[2] = 0.20725f;
        ambient[3] = 0.922f;
        // Diffuse
        diffuse[0] = 1.0f;
        diffuse[1] = 0.829f;
        diffuse[2] = 0.829f;
        diffuse[3] = 0.922f;
        // Specular
        specular[0] = 0.296648f;
        specular[1] = 0.296648f;
        specular[2] = 0.296648f;
        specular[3] = 0.922f;
        // Shininess
        shininess[0] = 11.264f;
        break;
    case Material::Ruby:
        // Ambient
        ambient[0] = 0.1745f;
        ambient[1] = 0.01175f;
        ambient[2] = 0.01175f;
        ambient[3] = 0.55f;
        // Diffuse
        diffuse[0] = 0.61424f;
        diffuse[1] = 0.04136f;
        diffuse[2] = 0.04136f;
        diffuse[3] = 0.55f;
        // Specular
        specular[0] = 0.727811f;
        specular[1] = 0.626959f;
        specular[2] = 0.626959f;
        specular[3] = 0.55f;
        // Shininess
        shininess[0] = 76.8f;
        break;
    case Material::Turquoise:
        // Ambient
        ambient[0] = 0.1f;
        ambient[1] = 0.18725f;
        ambient[2] = 0.1745f;
        ambient[3] = 0.8f;
        // Diffuse
        diffuse[0] = 0.396f;
        diffuse[1] = 0.74151f;
        diffuse[2] = 0.69102f;
        diffuse[3] = 0.8f;
        // Specular
        specular[0] = 0.297254f;
        specular[1] = 0.30829f;
        specular[2] = 0.306678f;
        specular[3] = 0.8f;
        // Shininess
        shininess[0] = 12.8f;
        break;
    case Material::Brass:
        // Ambient
        ambient[0] = 0.329412f;
        ambient[1] = 0.223529f;
        ambient[2] = 0.027451f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.780392f;
        diffuse[1] = 0.268627f;
        diffuse[2] = 0.113725f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.992157f;
        specular[1] = 0.741176f;
        specular[2] = 0.807843f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 27.8974f;
        break;
    case Material::Default:
        // Ambient
        ambient[0] = 0.2f;
        ambient[1] = 0.2f;
        ambient[2] = 0.2f;
        ambient[3] = 1.0f;
        // Diffuse
        diffuse[0] = 0.8f;
        diffuse[1] = 0.8f;
        diffuse[2] = 0.8f;
        diffuse[3] = 1.0f;
        // Specular
        specular[0] = 0.f;
        specular[1] = 0.f;
        specular[2] = 0.f;
        specular[3] = 1.0f;
        // Shininess
        shininess[0] = 0.f;
        break;
    default:
        throw "error";
    }

    // apply
    glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, shininess);
    glMaterialfv(GL_FRONT, GL_EMISSION, emission);
}
