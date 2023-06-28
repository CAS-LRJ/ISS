struct MapCoords{
  int x;
  int y;
  double cost;

  MapCoords(){
    this->x = -1;
    this->y = -1;
    this->cost = -1;
  }

  MapCoords(int x, int y, double cost){
    this->x = x;
    this->y = y;
    this->cost = cost;
  }

  bool operator<(const MapCoords &coords) const {
    return cost > coords.cost;
  }
};