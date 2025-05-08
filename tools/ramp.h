#pragma once
#include <algorithm>

class Ramp {
    unsigned char m_colors[4][256];
    int m_nodes[256];
    double min_value, max_value;
    bool cyclic;
    bool saturation;

  public:
    // life cycle
    Ramp() {
        cyclic = true;
        min_value = 0.;
        max_value = 1.;
        build_rainbow();
    }

    enum ColorMap { RAINBOW, THERMAL, FIRE, INFRARED, LIGHT_RAINBOW, MULTICOLOR, RANDOM_PASTEL, MENTHOL, FRANCE };

    unsigned char *color(unsigned int index) { return m_colors[index % 256]; }
    unsigned char r(unsigned int index) const { return m_colors[0][index % 256]; }
    unsigned char g(unsigned int index) const { return m_colors[1][index % 256]; }
    unsigned char b(unsigned int index) const { return m_colors[2][index % 256]; }

    unsigned int range_index(double value) const {
        const double range = max_value - min_value;
        int index = int(255.0 * (value - min_value) / range);
        if (!cyclic) {
            index = std::min(std::max(index, 0), 255);
        }
        return index;
    }

    void gl_color(const double value) const;
    void gl_color(const double value, double minv, double maxv) const;

    void set_cyclic(bool b) { cyclic = b; }
    void set_saturation(bool b) { saturation = b; }
    void set_range(double minv, double maxv) {
        min_value = minv;
        max_value = maxv;
    }
    void get_range(double &minv, double &maxv) const {
        minv = min_value;
        maxv = max_value;
    }

  private:
    void rebuild();

    void reset();

  public:
    // predefined
    void build_default();

    void add_node(unsigned int index, unsigned char r, unsigned char g, unsigned char b);

    void build_colormap(ColorMap colormap);

    void build_menthol();

    void build_fire();

    void build_rainbow();

    void build_infrared();

    void build_light_rainbow();

    void build_thermal();

    void build_multicolor();

    void build_random_pastel();

    void build_france();
};
