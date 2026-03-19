#pragma once

struct Node
{
    int id;
    double x;
    double y;

    Node(int id_, double x_, double y_)
        : id(id_), x(x_), y(y_) {
    }
};