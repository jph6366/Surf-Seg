#ifndef TRIANGLE_H_
#define TRIANGLE_H_

#include <map>
#include <vector>
#include <algorithm>
#include <cmath>

struct Vertex {
    float x, y, z;

    Vertex(float x, float y, float z) : x(x), y(y), z(z) {}

    bool operator==(const Vertex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    float length() const {

        return std::sqrt(x * x + y * y + z * z);

    }



    Vertex operator-(const Vertex& other) const {

        return Vertex(x - other.x, y - other.y, z - other.z);

    }

    float dot(const Vertex& other) const {
    return x * other.x + y * other.y + z * other.z;
    }

    float distanceTo(const Vertex& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        float dz = z - other.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};


struct Triangle {
    Vertex v1, v2, v3;

    Triangle(const Vertex& v1, const Vertex& v2, const Vertex& v3)
        : v1(v1), v2(v2), v3(v3) {}     

    bool sharesVertexWith(const Triangle& other, float radius = 0.0025f) const {
        auto isWithinRadius = [radius](const Vertex& v1, const Vertex& v2) {
            return (v1 - v2).length() <= radius;
        };

        return (v1 == other.v1 || v1 == other.v2 || v1 == other.v3 || isWithinRadius(v1, other.v1) || isWithinRadius(v1, other.v2) || isWithinRadius(v1, other.v3)) ||
                (v2 == other.v1 || v2 == other.v2 || v2 == other.v3 || isWithinRadius(v2, other.v1) || isWithinRadius(v2, other.v2) || isWithinRadius(v2, other.v3)) ||
                (v3 == other.v1 || v3 == other.v2 || v3 == other.v3 || isWithinRadius(v3, other.v1) || isWithinRadius(v3, other.v2) || isWithinRadius(v3, other.v3));
    }

    Vertex getNormal() const {
        Vertex u(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
        Vertex v(v3.x - v1.x, v3.y - v1.y, v3.z - v1.z);
        return Vertex(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x);
    }


};

#endif