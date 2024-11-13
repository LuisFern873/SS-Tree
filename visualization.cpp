#include <iostream>
#include <vector>
#include <unordered_set>
#include <random>
#include "Point.h"
#include "Data.h"
#include "SSTree.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

constexpr size_t NUM_POINTS = 100;
constexpr size_t MAX_POINTS_PER_NODE = 20;

void drawCircle(double x_center, double y_center, double radius) {
    const int num_points = 100;
    std::vector<double> x_values, y_values;

    for (int i = 0; i < num_points; ++i) {
        double theta = 2.0 * M_PI * i / num_points;
        x_values.push_back(x_center + radius * std::cos(theta));
        y_values.push_back(y_center + radius * std::sin(theta));
    }

    plt::plot(x_values, y_values, "b");
}

void plot(SSNode* node) {

    double x = node->getCentroid().coordinates_[0];
    double y = node->getCentroid().coordinates_[1];
    double radius = node->getRadius();

    // std::cout << "centroid = (" << x << "," << y << ")" << std::endl;
    // std::cout << "radius = " << radius << std::endl;

    plt::plot({x}, {y}, "bo");
    drawCircle(x, y, radius);

    if (node->getIsLeaf()) {
        for (const auto& data : node->getData()) {
            plt::plot({data->getEmbedding().coordinates_[0]}, {data->getEmbedding().coordinates_[1]}, "ro");
        }
    }

    for (SSNode* child : node->getChildren()) {
        plot(child);
    }
}

void plot(const SSTree& tree) {
    if (tree.getRoot() == nullptr) return;
    plot(tree.getRoot());

    plt::title("SS-tree Visualization 2D (NUM_POINTS = 100)");
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::xlim(-0.5, 1.5);
    plt::ylim(-0.5, 1.5);
    plt::grid(true);
    plt::show();
}

std::vector<Data*> generateRandomData(size_t numPoints) {
    std::vector<Data*> data;
    for (size_t i = 0; i < numPoints; ++i) {
        Point embedding = Point::random();
        std::string imagePath = "eda_" + std::to_string(i) + ".jpg";
        Data* dataPoint = new Data(embedding, imagePath);
        data.push_back(dataPoint);
    }
    return data;
}

int main() {
    auto data = generateRandomData(NUM_POINTS);
    SSTree tree(MAX_POINTS_PER_NODE);
    for (const auto& d : data) {
        tree.insert(d);
    }

    plot(tree);

    // Realizar pruebas
    // bool allPresent     = allDataPresent(tree, data);
    // bool sameLevel      = leavesAtSameLevel(tree.getRoot());
    // bool noExceed       = noNodeExceedsMaxChildren(tree.getRoot(), MAX_POINTS_PER_NODE);
    // bool spherePoints   = sphereCoversAllPoints(tree.getRoot());
    // bool sphereChildren = sphereCoversAllChildrenSpheres(tree.getRoot());
    
    // std::cout << "Todos los datos presentes: " << (allPresent ? "Sí" : "No") << std::endl;
    // std::cout << "Nodos hojas en el mismo nivel: " << (sameLevel ? "Sí" : "No") << std::endl;
    // std::cout << "No se supera el límite de hijos por nodo: " << (noExceed ? "Sí" : "No") << std::endl;
    // std::cout << "Hiper-esfera cubre todos los puntos de los nodos hoja: " << (spherePoints ? "Sí" : "No") << std::endl;
    // std::cout << "Hiper-esfera cubre todas las hiper-esferas internas de los nodos internos: " << (sphereChildren ? "Sí" : "No") << std::endl;
    // std::cout << "Happy ending! :D" << std::endl;

    return 0;
}

