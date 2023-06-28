struct Record{
  int x;
  int y;
  int yaw;
  double x_;
  double y_;
  double yaw_;
  double cost;
  double sum_cost;

  Record(){
    this->x = -1;
    this->y = -1;
    this->yaw = -1;
    this->x_ = -1;
    this->y_ = -1;
    this->yaw_ = -1;
    this->cost = -1;
    this->sum_cost = -1;
  }

  Record(int x, int y, int yaw, double x_, double y_, double yaw_, double cost, double sum_cost){
    this->x = x;
    this->y = y;
    this->yaw = yaw;
    this->x_ = x_;
    this->y_ = y_;
    this->yaw_ = yaw_;
    this->cost = cost;
    this->sum_cost = sum_cost;
  }

  bool operator<(const Record &record) const {
    return sum_cost > record.sum_cost;
  }
};