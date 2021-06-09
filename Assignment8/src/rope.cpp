#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        //        Comment-in this part when you implement the constructor
        // std::cout << node_mass <<" " << k << std::endl;
        Vector2D step = (end - start) / (num_nodes - 1);
        int j;
        Vector2D node_pos;
        for (j = 0, node_pos = start; j < num_nodes; j++, node_pos += step){
            masses.emplace_back(new Mass(node_pos, node_mass, false));
        }
        // for (j = 0; j < num_nodes; j++){
        //     masses[j]->forces = Vector2D(0, 0);
        //     masses[j]->forces = Vector2D(0, 0);
        // }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
        for (j = 0; j < num_nodes - 1; j++){
            springs.emplace_back(new Spring(masses[j], masses[j + 1], k));
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        Vector2D f, fb;
        // std::cout << delta_t <<"delta_t"<< std::endl;
        // static int a = 0;
        // a++;
        // std::cout << "step:" << a << std::endl;
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            f = s->m2->position - s->m1->position;
            fb = s->m2->velocity - s->m1->velocity;
            fb = 0.05 * dot(f, fb) * f / f.norm2();
            f = f * (1 - s->rest_length / f.norm()) * s->k;
            // std::cout << "force" << f.x <<" "<< f.y << std::endl;
            s->m1->forces += f + fb;
            s->m2->forces -= f + fb;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity;
                // std::cout << "mass force" << m->forces.x <<" "<< m->forces.y << std::endl;
                // std::cout << "new position" << m->position.x <<" "<< m->position.y << std::endl;
                // std::cout << "new velocity" << m->velocity.x <<" "<< m->velocity.y << std::endl;
                // TODO (Part 2): Add global damping
                m->velocity += m->forces / m->mass * delta_t * (1 - 0.0005);
                m->position += delta_t * m->velocity;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        Vector2D f, fb;
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            f = s->m2->position - s->m1->position;
            fb = s->m2->velocity - s->m1->velocity;
            fb = 0.05 * dot(f, fb) * f / f.norm2();
            f = f * (1 - s->rest_length / f.norm()) * s->k;
            // std::cout << "force" << f.x <<" "<< f.y << std::endl;
            s->m1->forces += f + fb;
            s->m2->forces -= f + fb;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                m->forces += gravity;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                m->position = temp_position + (1 - 0.00005) * (temp_position - m->last_position) + m->forces * delta_t * delta_t;
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
