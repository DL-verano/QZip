#pragma once

enum class Material {
    Silver,
    Gold,
    Jade,
    LightBlue,
    Emerald,
    PolishedSilver,
    Chrome,
    Copper,
    PolishedGold,
    Pewter,
    Obsidian,
    BlackPlastic,
    PolishedBronze,
    PolishedCopper,
    Pearl,
    Ruby,
    Turquoise,
    Brass,
    Default,
};

void gl_set_material(Material m);