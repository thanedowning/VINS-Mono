#pragma once

class Int3DPoint {
public:
  int x;
  int y;
  int z;
  inline bool operator==(const Int3DPoint& rhs) const {return ((x==rhs.x)&&(y==rhs.y)&&(z==rhs.z));}
  inline bool operator!=(const Int3DPoint& rhs) const {return !operator==(rhs);}
};
