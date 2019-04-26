#include <vector>
#include <memory>
#include <algorithm>
#include "bahamut.h"
#include "utils.h"

//
//   CONSTANTS
//

const i32 WALL_HP = 100;
const i32 MINE_COST = 50;

const f32 MAX_SPEED = 2;
const f32 MAX_FORCE = 4.5;
const f32 MIN_SEPERATION = 18;
const f32 SCALING_FACTOR = (1/15.0f);
const f32 VELOCITY_MINIMUM = 0.20;

//
//   DATA TYPES
//

struct Unit {
    vec2 pos;
    vec2 velocity;
    vec2 force;
    std::vector<vec2> path;
};
typedef std::vector<Unit> UnitList;

struct Map {
    i32* grid;
    u16 width;
    u16 height;
    std::vector<Unit> units;
};

struct Node;
typedef std::shared_ptr<Node> NodePtr;
struct Node {
    Node(i32 x, i32 y, NodePtr parent, i32 fcost) {
        this->x = x;
        this->y = y;
        this->parent = parent;
        this->fcost = fcost;
    }
    i32 x;
    i32 y;
    NodePtr parent;
    i32 fcost;
};

//
//   HELPER FUNCTIONS
//

static inline
void truncate(vec2* v, f32 max) {
    f32 i = 0;
    i = max / length(*v);
    i = i < 1.0 ? i : 1.0;
    v->x *= i;
    v->y *= i;
}

static inline
void clamp(i32* in, i32 min, i32 max) {
    if(*in > max) *in = max;
    if(*in < min) *in = min;
}

static inline
void clamp(f32* in, f32 min, f32 max) {
    if(*in > max) *in = max;
    if(*in < min) *in = min;
}

static inline
bool blocked_tile(i32 id) {
    if(id == 12)
        return true;
    return false;
}

//
//   PATH STEERING
//

static inline
vec2 calculate_seek(vec2 dest, Unit* unit) {
    vec2 desired = MAX_SPEED * normalize(dest - unit->pos);
    return desired - unit->velocity;
}

static inline
vec2 calculate_seperation(Map* map, UnitList* list, Unit* unit) {
    vec2 total = {0};

    for(u32 i = 0; i < list->size(); ++i) {
        Unit* a = &(*list)[i];
        if(a != unit) {
            f32 dist = getDistanceE(unit->pos.x, unit->pos.y, a->pos.x, a->pos.y);
            if(dist < MIN_SEPERATION && dist >= 0) {
                vec2 push = unit->pos - a->pos;
                total = total + (push/15.0f);
            }
        }
    }

    for(u32 x = 0; x < map->width; ++x) {
        for(u32 y = 0; y < map->height; ++y) {
            u32 index = x + y * map->width;

            if(blocked_tile(map->grid[index])) {
                f32 dist = getDistanceE(unit->pos.x, unit->pos.y, x * 64, y * 64);
                if(dist < 50 && dist >= 0) {
                    vec2 push = unit->pos - V2(x * 64, y * 64);
                    total = total + push;
                }
            }
        }
    }
    return MAX_FORCE * total;
}

//
//   PATHFINDING
//
//

static inline
bool compare_ptr_to_node(NodePtr a, NodePtr b) {
    return (a->fcost > b->fcost);
}

static inline
void process_successor(i32 x, i32 y, i32 fcost, NodePtr parent, std::vector<NodePtr>& open, std::vector<NodePtr>& closed) {
    if(x < 0 || y < 0) return;

    for(u16 i = 0; i < open.size(); ++i) {
        i32 nodex = open[i]->x;
        i32 nodey = open[i]->y;
        if(nodex == x && nodey == y) {
            if(open[i]->fcost <= fcost)
                return;
            break;
        }
    }

    for(u16 i = 0; i < closed.size(); ++i) {
        i32 nodex = closed[i]->x;
        i32 nodey = closed[i]->y;
        if(nodex == x && nodey == y) {
            if(closed[i]->fcost <= fcost)
                return;
            break;
        }
    }
    open.push_back(NodePtr(new Node(x, y, parent, fcost)));
}

static inline
std::vector<vec2> reconstruct_path(NodePtr current) {
    std::vector<vec2> path;
    while(current->parent != NULL) {
        path.push_back({(f32)current->x, (f32)current->y});
        current = current->parent;
    }
    path.push_back({(f32)current->x, (f32)current->y});
    return path;
}

static inline
bool jump_search_horizontal(Map* map, NodePtr current, std::vector<vec2>& successors, i32 dx, vec2 dest) {
    vec2 currpt = {current->x, current->y};

    for(;;) {
        currpt.x+=dx;
        //if reached destination
        if(currpt.x == dest.x && currpt.y == dest.y) {
            successors.push_back(currpt);
            return false;
        }
        //if a wall or edge of map reached, exit
        u32 index3 = currpt.x + currpt.y * map->width;
        if(currpt.x < 0 || currpt.x > map->width-1 || blocked_tile(map->grid[index3]))
            return true;

        //if there are forced neighbors up or down, add successor and exit
        u32 index = currpt.x + (currpt.y-1) * map->width;
        u32 index2 = currpt.x + (currpt.y+1) * map->width;
        if(currpt.y - 1 < 0 || currpt.y + 1 > map->height-1) {
            successors.push_back(currpt);
            return false;
        }
        if(blocked_tile(map->grid[index]) || blocked_tile(map->grid[index2])) {
            successors.push_back({currpt.x, currpt.y});
            return false;
        }
    }
}

static inline
bool jump_search_vertical(Map* map, NodePtr current, std::vector<vec2>& successors, i32 dy, vec2 dest) {
    vec2 currpt = {current->x, current->y};

    for(;;) {
        currpt.y+=dy;
        //if reached destination
        if(currpt.x == dest.x && currpt.y == dest.y) {
            successors.push_back(currpt);
            return false;
        }
        //if a wall or edge of map reached, exit
        u32 index3 = currpt.x + currpt.y * map->width;
        if(currpt.y < 0 || currpt.y > map->height-1 || blocked_tile(map->grid[index3]))
            return true;

        //if there are forced neighbors left or right, add successor and exit
        u32 index = (currpt.x-1) + currpt.y * map->width;
        u32 index2 = (currpt.x+1) + currpt.y * map->width;
        if(currpt.x - 1 < 0 || currpt.x + 1 > map->width-1) {
            successors.push_back(currpt);
            return false;
        }
        if(blocked_tile(map->grid[index]) || blocked_tile(map->grid[index2])) {
            successors.push_back({currpt.x, currpt.y});
            return false;
        }
    }
}

static inline
std::vector<vec2> find_successors(Map* map, NodePtr current, vec2 start, vec2 dest) {
    std::vector<vec2> successors;
    //go in all four cardinal directions, if all of them
    //hit walls, with no forced neighbors, then it is safe to traverse
    //diagonally.
    //if there are forced neighbors at any point (traversing diagonally
    //or cardinally), then add that node to the open list (successors in
    //this func) and continue with the A* processing.
    
    bool left_hit_wall = jump_search_horizontal(map, current, successors, -1, dest);
    bool right_hit_wall = jump_search_horizontal(map, current, successors, 1, dest);

    bool up_hit_wall = jump_search_vertical(map, current, successors, -1, dest);
    bool down_hit_wall = jump_search_vertical(map, current, successors,1, dest);

    if(right_hit_wall && up_hit_wall) {
        //SAFE to go up and right
        u32 index = (current->x + 1) + (current->y - 1) * map->width;
        if(current->x + 1 < map->width - 1 && current->y - 1 > 0 && !blocked_tile(map->grid[index]))
            successors.push_back({(f32)current->x + 1, (f32)current->y - 1});
    }

    if(up_hit_wall && left_hit_wall) {
        //SAFE to go up and left
        u32 index = (current->x - 1) + (current->y - 1) * map->width;
        if(current->x - 1 > 0 && current->y - 1 > 0 && !blocked_tile(map->grid[index]))
            successors.push_back({(f32)current->x - 1, (f32)current->y - 1});
    }

    if(left_hit_wall && down_hit_wall) {
        //SAFE to go down and left
        u32 index = (current->x - 1) + (current->y + 1) * map->width;
        if(current->x - 1 > 0 && current->y + 1 < map->height-1 && !blocked_tile(map->grid[index]))
            successors.push_back({(f32)current->x - 1, (f32)current->y + 1});
    }

    if(down_hit_wall && right_hit_wall) {
        //SAFE to go down and right
        u32 index = (current->x + 1) + (current->y + 1) * map->width;
        if(current->x + 1 < map->width-1 && current->y + 1 < map->height-1 && !blocked_tile(map->grid[index]))
            successors.push_back({(f32)current->x + 1, (f32)current->y + 1});
    }
    
    return successors;
}

static inline
std::vector<vec2> pathfind(Map* map, vec2 start, vec2 dest) {
    std::vector<NodePtr> open;
    std::vector<NodePtr> closed;
    NodePtr current;
    open.push_back(NodePtr(new Node(start.x, start.y, NULL, 0)));

    u32 tries = 0;
    while(!open.empty() && tries < 10000) {
        tries++;

        std::sort(open.begin(), open.end(), compare_ptr_to_node);
        current = open.back();
        open.pop_back();

        if(current->x == dest.x && current->y == dest.y)
            return reconstruct_path(current);

        //now use JUMP POINT SEARCH to eliminate nodes that can be skipped, and return new nodes
        std::vector<vec2> successors = find_successors(map, current, start, dest);
        for(u32 i = 0; i < successors.size(); ++i) {
            vec2 currpt = successors[i];
            process_successor(currpt.x, currpt.y, getDistanceE(currpt.x, currpt.y, dest.x, dest.y), current, open, closed);
        }

        closed.push_back(current);
    }

    std::vector<vec2> blank;
    blank.push_back(start);
    return blank;
}


static inline
std::vector<vec2> pathfind_astar(Map* map, vec2 start, vec2 dest) {
	if (start.x < 0 || start.y < 0 || start.x > map->width - 1 || start.y > map->height - 1)
		BMT_LOG(FATAL_ERROR, "vec2 start was out of bounds. start = (%f, %f)", start.x, start.y);
	if (dest.x < 0 || dest.y < 0 || dest.x > map->width - 1 || dest.y > map->height - 1)
		BMT_LOG(FATAL_ERROR, "vec2 dest was out of bounds. dest = (%f, %f)", dest.x, dest.y);

	std::vector<NodePtr> open;
	std::vector<NodePtr> closed;
	NodePtr current;
	open.push_back(NodePtr(new Node(start.x, start.y, NULL, 0)));

	u16 tries = 0;
	while (!open.empty() && tries < 650) {
		tries++;

		std::sort(open.begin(), open.end(), compare_ptr_to_node);
		current = open.back();
		open.pop_back();

		if (current->x == dest.x && current->y == dest.y) {
			return reconstruct_path(current);
		}

		//if heuristic cost = -1 then the tile is blocked and cant be move to.
		int x = current->x - 1;
		int y = current->y;
		if (x > map->width) continue;
		if (y > map->height) continue;
        if(!blocked_tile(map->grid[x + y * map->width]))
		    process_successor(x, y, 1 + getDistanceE(x, y, dest.x, dest.y), current, open, closed);
        else if(dest.x == x && dest.y == y)
            return reconstruct_path(current);

		x = current->x + 1;
		y = current->y;
		if (x > map->width) continue;
		if (y > map->height) continue;
        if(!blocked_tile(map->grid[x + y * map->width]))
		    process_successor(x, y, 1 + getDistanceE(x, y, dest.x, dest.y), current, open, closed);
        else if(dest.x == x && dest.y == y)
            return reconstruct_path(current);

		x = current->x;
		y = current->y - 1;
		if (x > map->width) continue;
		if (y > map->height) continue;
        if(!blocked_tile(map->grid[x + y * map->width]))
		    process_successor(x, y, 1 + getDistanceE(x, y, dest.x, dest.y), current, open, closed);
        else if(dest.x == x && dest.y == y)
            return reconstruct_path(current);

		x = current->x;
		y = current->y + 1;
		if (x > map->width) continue;
		if (y > map->height) continue;
        if(!blocked_tile(map->grid[x + y * map->width]))
		    process_successor(x, y, 1 + getDistanceE(x, y, dest.x, dest.y), current, open, closed);
        else if(dest.x == x && dest.y == y)
            return reconstruct_path(current);

		closed.push_back(current);
	}
	if (tries == 650) {
		return reconstruct_path(current);
	}
	std::vector<vec2> blank;
	blank.push_back(start);
	return blank;
}

//
//   MAP
//

static inline
Map load_test_map() {
    Map map = {0};

    map.width = map.height = 20;
    map.grid = (i32*)malloc(sizeof(i32) * (map.width * map.height));
    
    for(u32 i = 0; i < 20 * 20; ++i)
        map.grid[i] = 0;

    return map;
}

static inline
void draw_map(RenderBatch* batch, Map* map) {
    for(u32 x = 0; x < map->width; ++x) {
        for(u32 y = 0; y < map->height; ++y) {
            u32 index = map->grid[x + y * map->width];
            if(index == 0) {
                draw_rectangle(batch, x * 64, y * 64, 64, 64,  GRAY);
                draw_rectangle(batch, (x * 64)+2, (y * 64)+2, 60, 60, WHITE);
            }
            if(index == 12) {
                draw_rectangle(batch, x * 64, y * 64, 64, 64, GRAY);
                draw_rectangle(batch, (x * 64)+2, (y * 64)+2, 60, 60, DARKGRAY);
            }
        }
    }
}
