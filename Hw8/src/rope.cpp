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

        masses.resize(node_mass);masses.clear();
        springs.resize(node_mass-1);springs.clear();
        Vector2D dir = end - start;
        for(int i=0;i<num_nodes;++i){
            Vector2D pos = start+(dir*(double)i/((double)num_nodes-1.0f));
            masses.push_back(new Mass(pos,node_mass, false));
            if(i) springs.push_back(new Spring(masses[i-1],masses[i],k));
        }

//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            double dnorm = (s->m2->position - s->m1->position).norm();
            double delta = (dnorm - s->rest_length) * s->k;
            Vector2D F = (s->m2->position - s->m1->position) * delta / dnorm;
            s->m1->forces += F;
            s->m2->forces -= F;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // TODO (Part 2): Add global damping
                Vector2D a = (m->forces - m->velocity * m->velocity.norm() * damping_factor) / m->mass + gravity;
//                m->position += m->velocity * delta_t;
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            double dnorm = (s->m2->position - s->m1->position).norm();
            double delta = (dnorm - s->rest_length) * s->k;
            Vector2D F = (s->m2->position - s->m1->position) * delta / dnorm;
            s->m1->forces += F;
            s->m2->forces -= F;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                Vector2D temp_position = m->position;
                Vector2D a = m->forces / m->mass + gravity;
                m->position = temp_position + (temp_position - m->last_position) * (1 - damping_factor) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
