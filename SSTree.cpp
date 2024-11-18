#include "SSTree.h"
#include <utility>

/**
 * intersectsPoint
 * Verifica si un punto está dentro de la esfera delimitadora del nodo.
 * @param point: Punto a verificar.
 * @return bool: Retorna true si el punto está dentro de la esfera, de lo contrario false.
 */
bool SSNode::intersectsPoint(const Point& point) const {
    return Point::distance(centroid, point) <= radius;
}


/**
 * findClosestChild
 * Encuentra el hijo más cercano a un punto dado.
 * @param target: El punto objetivo para encontrar el hijo más cercano.
 * @return SSNode*: Retorna un puntero al hijo más cercano.
 */
SSNode* SSNode::findClosestChild(const Point& target) {

    if (isLeaf) {
        throw std::logic_error("El nodo es una hoja y no tiene hijos.");
    }

    float minDistance = std::numeric_limits<float>::infinity();
    SSNode* closestChild = nullptr;

    // Iteramos sobre cada hijo para encontrar el más cercano
    for (SSNode* child : children) {
        float dist = Point::distance(child->getCentroid(), target);
        
        if (dist < minDistance) {
            minDistance = dist;
            closestChild = child;
        }
    }

    return closestChild;
}


/**
 * updateBoundingEnvelope
 * Actualiza el centroide y el radio del nodo basándose en los nodos internos o datos.
 */

void SSNode::updateBoundingEnvelope() {
    // Obtener los puntos de los elementos contenidos en el nodo
    std::vector<Point> points = getEntriesCentroids();
    
    if (points.empty()) {
        throw std::logic_error("No hay puntos para actualizar el bounding envelope.");
    }

    // Calcular el nuevo centroide como la  mediade los puntos en cada dimensión
    Eigen::VectorXf centroidCoordinates = Eigen::VectorXf::Zero(DIM);

    for (const auto& point : points) {
        centroidCoordinates += point.coordinates_; // Sumamos cada punto
    }
    
    centroidCoordinates /= points.size(); // Dividimos por el número de puntos para obtener la media
    centroid = Point(centroidCoordinates); // Asignamos el nuevo centroide

    // Calcular el radio como la máxima distancia desde el nuevo centroide a cualquier punto en el nodo
    radius = 0.0f;
    for (const auto& entry : points) {
        float dist = Point::distance(centroid, entry);
        radius = std::max(radius, dist);
    }


    if (!isLeaf) {
        for (SSNode* child : children) {
            float dist = Point::distance(centroid, child->getCentroid()) + child->getRadius();
            radius = std::max(radius, dist);
        }
    }
}


float SSNode::varianceAlongDirection(const std::vector<Point>& centroids, size_t directionIndex) {
    if (centroids.empty()) {
        throw std::logic_error("No hay centroides para calcular la varianza.");
    }

    float mean = 0.0f;
    for (const Point& point : centroids) {
        mean += point[directionIndex];
    }

    mean /= centroids.size();

    // Calcula la varianza a lo largo de la dirección especificada
    float variance = 0.0f;
    for (const auto& point : centroids) {
        float diff = point[directionIndex] - mean;
        variance += diff * diff;
    }
    variance /= centroids.size();

    return variance;
}

/**
 * directionOfMaxVariance
 * Calcula y retorna el índice de la dirección de máxima varianza.
 * @return size_t: Índice de la dirección de máxima varianza.
 */
size_t SSNode::directionOfMaxVariance() {
    float maxVariance = 0.0f;
    size_t directionIndex = 0;

    std::vector<Point> centroids = getEntriesCentroids();

    for (size_t i = 0; i < DIM; ++i) {
        float variance = varianceAlongDirection(centroids, i);
        // std::cout << "Variance DIM " << i << " = " << variance << std::endl;

        if (variance > maxVariance) {
            maxVariance = variance;
            directionIndex = i;
        }
    }

    return directionIndex;
}


/**
 * split
 * Divide el nodo y retorna el nuevo nodo creado.
 * Implementación similar a R-tree.
 * @return SSNode*: Puntero al nuevo nodo creado por la división.
 */
std::pair<SSNode*, SSNode*> SSNode::split() {

    size_t splitIndex = findSplitIndex(directionOfMaxVariance());


    SSNode* newNode1 = nullptr;
    SSNode* newNode2 = nullptr;

    if (isLeaf) {
        std::vector<Data*> points1(_data.begin(), _data.begin() + splitIndex);
        std::vector<Data*> points2(_data.begin() + splitIndex, _data.end());

        newNode1 = new SSNode(centroid, maxPointsPerNode, radius, true, this);
        newNode2 = new SSNode(centroid, maxPointsPerNode, radius, true, this);

        newNode1->_data = points1;
        newNode2->_data = points2;

    } else {
        std::vector<SSNode*> children1(children.begin(), children.begin() + splitIndex);
        std::vector<SSNode*> children2(children.begin() + splitIndex, children.end());

        newNode1 = new SSNode(centroid, maxPointsPerNode, radius, false, this);
        newNode2 = new SSNode(centroid, maxPointsPerNode, radius, false, this);

        newNode1->children = children1;
        newNode2->children = children2;
    }

    newNode1->updateBoundingEnvelope();
    newNode2->updateBoundingEnvelope();

    return std::make_pair(newNode1, newNode2);
}


/**
 * findSplitIndex
 * Encuentra el índice de división en una coordenada específica.
 * @param coordinateIndex: Índice de la coordenada para encontrar el índice de división.
 * @return size_t: Índice de la división.
 */
size_t SSNode::findSplitIndex(size_t coordinateIndex) {
    
    std::vector<Point> centroids = getEntriesCentroids();
    
    // Ordenamos los puntos de acuerdo con la coordenada indicada
    std::sort(centroids.begin(), centroids.end(), [coordinateIndex](const Point& a, const Point& b) {
        return a[coordinateIndex] < b[coordinateIndex];
    });

    // Paso 2: Extraer los valores de los centroides en la dimensión especificada
    std::vector<float> points;
    for (const Point& point : centroids) {
        points.push_back(point[coordinateIndex]);
    }

    // Paso 3: Llamar a minVarianceSplit para encontrar el índice de división óptimo
    return minVarianceSplit(points);
}



/**
 * getEntriesCentroids
 * Devuelve los centroides de las entradas.
 * Estos centroides pueden ser puntos almacenados en las hojas o los centroides de los nodos hijos en los nodos internos.
 * @return std::vector<Point>: Vector de centroides de las entradas.
 */
std::vector<Point> SSNode::getEntriesCentroids() {

    std::vector<Point> entries;
    if (isLeaf) {
        entries.reserve(_data.size());
        std::transform(_data.begin(), _data.end(), std::back_inserter(entries),
                       [](Data* data) { return data->getEmbedding(); });
    } else {
        entries.reserve(children.size());
        std::transform(children.begin(), children.end(), std::back_inserter(entries),
                       [](SSNode* child) { return child->getCentroid(); });
    }
    return entries;
}



/**
 * minVarianceSplit
 * Encuentra el índice de división óptimo para una lista de valores, de tal manera que la suma de las varianzas de las dos particiones resultantes sea mínima.
 * @param values: Vector de valores para encontrar el índice de mínima varianza.
 * @return size_t: Índice de mínima varianza.
 */
size_t SSNode::minVarianceSplit(const std::vector<float>& values) {
    float minVariance = std::numeric_limits<float>::infinity();
    size_t splitIndex = values.size() / 2; // Valor inicial de splitIndex

    // Calcular la varianza total del conjunto de valores
    auto variance = [](const std::vector<float>& subValues) {
        if (subValues.empty()) return 0.0f;
        
        float mean = 0.0f;
        for (float value : subValues) {
            mean += value;
        }
        mean /= subValues.size();

        float var = 0.0f;
        for (float value : subValues) {
            var += (value - mean) * (value - mean);
        }
        return var / subValues.size();
    };

    // Recorrer todos los puntos de división posibles, desde m hasta |values| - m
    for (size_t i = values.size() / 2; i < values.size() - values.size() / 2; ++i) {
        // Particionar los valores en dos segmentos
        std::vector<float> left(values.begin(), values.begin() + i);
        std::vector<float> right(values.begin() + i, values.end());

        // Calcular la varianza para cada partición
        float variance1 = variance(left);
        float variance2 = variance(right);

        // Si la suma de las varianzas es menor que el mínimo encontrado, actualizar minVariance
        if (variance1 + variance2 < minVariance) {
            minVariance = variance1 + variance2;
            splitIndex = i;
        }
    }

    return splitIndex;
}


/**
 * searchParentLeaf
 * Busca el nodo hoja adecuado para insertar un punto.
 * @param node: Nodo desde el cual comenzar la búsqueda.
 * @param target: Punto objetivo para la búsqueda.
 * @return SSNode*: Nodo hoja adecuado para la inserción.
 */
SSNode* SSNode::searchParentLeaf(SSNode* node, const Point& target) {
    if (isLeaf) {
        return node;
    }
    SSNode* child = findClosestChild(target);
    return searchParentLeaf(child, target);
}

/**
 * insert
 * Inserta un dato en el nodo, dividiéndolo si es necesario.
 * @param node: Nodo donde se realizará la inserción.
 * @param _data: Dato a insertar.
 * @return SSNode*: Nuevo nodo raíz si se dividió, de lo contrario nullptr.
 */


std::pair<SSNode*, SSNode*> SSNode::insert(Data* data) {

    if (isLeaf) {
        _data.push_back(data);
        updateBoundingEnvelope();

        if (_data.size() <= maxPointsPerNode) {
            return {nullptr, nullptr};
        } 
    } else {
        
        SSNode* closestChild = findClosestChild(data->getEmbedding());

        auto [newChild1, newChild2] = closestChild->insert(data);

        if (newChild1 == nullptr) {
            updateBoundingEnvelope();
            return {nullptr, nullptr};
        } else {
            // Si el hijo se dividió, actualizamos los hijos del nodo actual
            children.erase(std::remove(children.begin(), children.end(), closestChild), children.end());
            children.push_back(newChild1);
            children.push_back(newChild2);

            updateBoundingEnvelope();  // Actualizamos la envoltura del nodo

            // Si el número de hijos no excede el límite, no es necesario dividir este nodo
            if (children.size() <= maxPointsPerNode) {
                return {nullptr, nullptr};  // No se dividió
            }
        }
    }
    // std::cout << "split()" << std::endl;
    return split();  // Retorna el par de nuevos nodos hijos
}



/**
 * search
 * Busca un dato específico en el árbol.
 * @param node: Nodo desde el cual comenzar la búsqueda.
 * @param _data: Dato a buscar.
 * @return SSNode*: Nodo que contiene el dato (o nullptr si no se encuentra).
 */
SSNode* SSNode::search(Data* data) {


    return nullptr;
}











/**
 * insert
 * Inserta un dato en el árbol.
 * @param _data: Dato a insertar.
 */
void SSTree::insert(Data* data) {
    if (root == nullptr) {
        root = new SSNode(data->getEmbedding(), maxPointsPerNode);
    }
    auto [newChild1, newChild2] = root->insert(data);

    if (newChild1 != nullptr) {
        root = new SSNode(data->getEmbedding(), maxPointsPerNode);
        root->children.push_back(newChild1);
        root->children.push_back(newChild2);
        root->isLeaf = false;
    }
}

/**
 * search
 * Busca un dato específico en el árbol.
 * @param _data: Dato a buscar.
 * @return SSNode*: Nodo que contiene el dato (o nullptr si no se encuentra).
 */
SSNode* SSTree::search(Data* _data) {
    return nullptr;
}



// Depth-Firstk-NearestNeighbor

/*
root->getData().size() = 0
root->getEntriesCentroids().size() = 4
root->getChildren()[0]->getData().size() = 0
*/

void collectData(SSNode* node, std::vector<Data*>& treeData) {
    if (node->getIsLeaf()) {
        for (const auto& d : node->getData()) {
            treeData.push_back(d);
        }
    } else {
        for (const auto& child : node->getChildren()) {
            collectData(child, treeData);
        }
    }
}


std::vector<Data*> SSTree::knn(Point point, size_t k) const {

    // Descartar nodos

    std::vector<Data*> result;
    collectData(root, result);

    std::sort(result.begin(), result.end(), [&point](Data *a, Data *b) {
        return a->getEmbedding().distance(point) < b->getEmbedding().distance(point);
    });

    result.resize(k);

    return result;
}