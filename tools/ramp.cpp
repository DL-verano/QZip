#include "ramp.h"

#include "include_opengl.h"
#include <algorithm>

void Ramp::gl_color(const double value) const { gl_color(value, min_value, max_value); }

void Ramp::gl_color(const double value, double minv, double maxv) const {
    const double range = maxv - minv;
    int index = int(255.0 * (value - minv) / range);
    if (!saturation) {
        if (!cyclic) {
            index = std::min(std::max(index, 0), 255);
        }
        ::glColor3ub(r(index), g(index), b(index));
    } else {
        if (value <= minv)
            ::glColor3ub(255, 255, 255);
        else if (value >= maxv)
            ::glColor3ub(255, 0, 0);
        else
            ::glColor3ub(r(index), g(index), b(index));
    }
}

void Ramp::rebuild() {
    // build nodes
    m_colors[3][0] = 1;
    m_colors[3][255] = 1;
    unsigned int nb_nodes = 0;
    for (int i = 0; i < 256; i++)
        if (m_colors[3][i]) {
            m_nodes[nb_nodes] = i;
            nb_nodes++;
        }
    // build ramp
    for (int k = 0; k < 3; k++)
        for (unsigned int i = 0; i < (nb_nodes - 1); i++) {
            int x1 = m_nodes[i];
            int x2 = m_nodes[i + 1];
            int y1 = m_colors[k][x1];
            int y2 = m_colors[k][x2];
            float a = float(y2 - y1) / float(x2 - x1);
            float b = float(y1) - a * float(x1);
            for (int j = x1; j < x2; j++)
                m_colors[k][j] = (unsigned char)(a * float(j) + b);
        }
}

void Ramp::reset() {
    for (int i = 1; i <= 254; i++)
        m_colors[3][i] = 0;
    m_colors[3][0] = 1;
    m_colors[3][255] = 1;
}

void Ramp::build_default() {
    reset();
    add_node(0, 0, 0, 0);
    add_node(255, 255, 255, 255);
    rebuild();
}

void Ramp::add_node(unsigned index, unsigned char r, unsigned char g, unsigned char b) {
    m_colors[3][index] = 1;
    m_colors[0][index] = r;
    m_colors[1][index] = g;
    m_colors[2][index] = b;
}

void Ramp::build_colormap(ColorMap colormap) {
    switch (colormap) {
    case RAINBOW:
        build_rainbow();
        break;
    case THERMAL:
        build_thermal();
        break;
    case FIRE:
        build_fire();
        break;
    case INFRARED:
        build_infrared();
        break;
    case LIGHT_RAINBOW:
        build_light_rainbow();
        break;
    case MULTICOLOR:
        build_multicolor();
        break;
    case RANDOM_PASTEL:
        build_random_pastel();
        break;
    case MENTHOL:
        build_menthol();
        break;
    case FRANCE:
        build_france();
        break;
    }
}

void Ramp::build_menthol() {
    reset();
    add_node(0, 0, 0, 128);
    add_node(48, 0, 0, 254);
    add_node(176, 0, 255, 255);
    add_node(208, 128, 255, 255);
    add_node(255, 255, 255, 255);
    rebuild();
}

void Ramp::build_fire() {
    reset();
    add_node(0, 0, 0, 0);
    add_node(92, 255, 0, 0);
    add_node(185, 255, 255, 0);
    add_node(255, 255, 255, 0);
    rebuild();
}

void Ramp::build_rainbow() {
    reset();
    add_node(0, 0, 0, 0);
    add_node(32, 128, 0, 255);
    add_node(80, 0, 0, 255);
    add_node(112, 0, 255, 255);
    add_node(144, 0, 255, 0);
    add_node(176, 255, 255, 0);
    add_node(208, 255, 128, 0);
    add_node(255, 255, 0, 0); // 240 before
    // add_node(255, 255, 255, 255);
    rebuild();
}

void Ramp::build_infrared() {
    reset();
    add_node(0, 0, 0, 0);
    add_node(32, 0, 0, 128);
    add_node(64, 128, 0, 255);
    add_node(112, 255, 0, 255);
    add_node(160, 255, 128, 0);
    add_node(176, 255, 128, 0);
    add_node(208, 255, 255, 0);
    add_node(255, 255, 255, 255);
    rebuild();
}

void Ramp::build_light_rainbow() {
    reset();
    add_node(0, 128, 255, 255);
    add_node(64, 168, 255, 168);
    add_node(128, 255, 255, 179);
    add_node(192, 255, 175, 175);
    add_node(255, 255, 255, 255);
    rebuild();
}

void Ramp::build_thermal() {
    reset();
    add_node(0, 0, 0, 0);
    add_node(70, 128, 0, 0);
    add_node(165, 255, 128, 0);
    add_node(255, 255, 255, 255);
    rebuild();
}

void Ramp::build_multicolor() {
    reset();
    add_node(0, 255, 128, 192);
    add_node(32, 0, 0, 255);
    add_node(64, 0, 255, 255);
    add_node(96, 0, 252, 0);
    add_node(128, 255, 254, 0);
    add_node(176, 255, 128, 0);
    add_node(208, 255, 0, 0);
    add_node(240, 192, 192, 192);
    add_node(255, 255, 255, 255);
    rebuild();
}

void Ramp::build_random_pastel() {
    reset();
    for (int i = 0; i < 256; i += 5) {
        m_colors[3][i] = 1;
        m_colors[0][i] = (unsigned char)(double(rand()) / double(RAND_MAX) * 255.0);
        m_colors[1][i] = (unsigned char)(double(rand()) / double(RAND_MAX) * 255.0);
        m_colors[2][i] = (unsigned char)(double(rand()) / double(RAND_MAX) * 255.0);
    }
    rebuild();
}

void Ramp::build_france() {
    reset();
    int k = 0;
    for (int i = 0; i < 256; i += 17) {
        m_colors[3][i] = 1;
        switch (k) {
        case 0:

            m_colors[0][i] = 0;
            m_colors[1][i] = 0;
            m_colors[2][i] = 255;
            break;
        case 1:
            m_colors[0][i] = 255;
            m_colors[1][i] = 255;
            m_colors[2][i] = 255;
            break;
        case 2:
            m_colors[0][i] = 255;
            m_colors[1][i] = 0;
            m_colors[2][i] = 0;
            break;
        default:
            break;
        }

        k++;
        k %= 3;
    }
    rebuild();
}
