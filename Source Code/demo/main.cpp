#include "bahamut.h"
#include "map.h"

//This is going to be a jump-point-search pathfinding demo first, I'll upload that to
//github once it is functional, then work on the game will begin.

enum MainState {
    MAIN_TITLE,
    MAIN_DUNGEON,
    MAIN_OVERWORLD,
    MAIN_SUMMARY,
    MAIN_EXIT
};

int main() {
    init_window(800, 600, "Monster Manager", false, true, true);
    set_fps_cap(60);
    set_clear_color(SKYBLUE);
    set_vsync(true);

    RenderBatch * batch = &create_batch();
    Shader basic = load_default_shader_2D();
    MainState state = MAIN_TITLE;

    Map map = load_test_map();
    std::vector<Unit> units;
    for(u32 i = 0; i < 20; ++i) {
        Unit unit = {0};
        unit.pos.x = random_int(0, 600);
        unit.pos.y = random_int(0, 600);
        units.push_back(unit);
    }

    Texture attacker = load_texture("data/art/attacker1.png", GL_LINEAR);

    while(window_open()) {
        vec2 mouse = get_mouse_pos();
        set_viewport(0, 0, get_window_width(), get_window_height());

        begin_drawing();
        begin2D(batch, basic);
            upload_mat4(basic, "projection", orthographic_projection(0, 0, get_window_width(), get_window_height(), -1, 1));

            if(state == MAIN_TITLE) {
                draw_map(batch, &map);

                for(u32 i = 0; i < units[0].path.size(); ++i)
                    draw_rectangle(batch, units[0].path[i].x * 64, units[0].path[i].y * 64, 64, 64, {100, 255, 100, 100});

                if(is_button_down(MOUSE_BUTTON_LEFT)) {
                    i32 index = (i32)(mouse.x / 64) + (i32)(mouse.y / 64) * map.width;
                    if(map.grid[index] == 0)
                        map.grid[index] = 12;
                   // else if(map.grid[index] == 12)
                    
                     //   map.grid[index] = 0;
                }

                if(is_button_released(MOUSE_BUTTON_RIGHT)) {
                    i32 tilex = (i32)(mouse.x/64);
                    i32 tiley = (i32)(mouse.y/64);
                    for(u32 i = 0; i < units.size(); ++i)
                    //    units.at(i).path.push_back(mouse);
                    units.at(i).path = pathfind_astar(&map, {(f32)(units.at(i).pos.x / 64), (f32)(units.at(i).pos.y / 64)}, {(f32)tilex, (f32)tiley});
                }
                
                for(u16 i = 0; i < units.size(); ++i) {
                    Unit* unit = &units[i];
                    draw_texture(batch, attacker, unit->pos.x, unit->pos.y);
                    if(unit->path.size() > 0) {
                        vec2 seek = calculate_seek({(unit->path.back().x * 64)+32, (unit->path.back().y * 64)+32}, unit);
                        vec2 seperation = calculate_seperation(&map, &units, unit);
                        unit->force = seek + seperation;
                        truncate(&unit->force, MAX_FORCE);
                    }
                }
                
                for(u16 i = 0; i < units.size(); ++i) {
                    Unit* unit = &units[i];
                    unit->velocity = unit->velocity + (SCALING_FACTOR * unit->force);
                    truncate(&unit->velocity, MAX_SPEED);
                    unit->pos = unit->pos + unit->velocity;
                
                    if(unit->path.size() > 0 && getDistanceE(unit->pos.x, unit->pos.y, (unit->path.back().x * 64) + 32, (unit->path.back().y * 64) + 32) < 44) {
                        if(unit->path.size() > 1)
                            unit->path.pop_back();

                        if(abs(unit->velocity.x) < VELOCITY_MINIMUM && abs(unit->velocity.y) < VELOCITY_MINIMUM && unit->path.size() == 1) {
                            unit->path.clear();
                            unit->velocity = {0, 0};
                            unit->force = {0, 0};
                        }
                    }
                }

            }
            if(state == MAIN_EXIT) 
                exit(0);
            
        end2D(batch);
        end_drawing();
    }

    dispose_batch(batch);
    dispose_window();

    return 0;
}
