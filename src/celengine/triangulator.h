// not sure we need this
#include <celmath/vecmath.h>
#include <vector>

typedef std::vector<Point2f> Vector2dVector;

class Triangulate
{
public:
    // triangulate a contour/polygon, places results in STL vector
    // as series of triangles.
    static bool Process(const Vector2dVector &contour, Vector2dVector &result);

    // compute area of a contour/polygon
    static float Area(const Vector2dVector &contour);

    // decide if point Px/Py is inside triangle defined by
    // (Ax,Ay) (Bx,By) (Cx,Cy)
    static bool
    InsideTriangle(float Ax, float Ay, float Bx, float By, float Cx, float Cy, float Px, float Py);

private:
    static bool Snip(const Vector2dVector &contour, int u, int v, int w, int n, int *V);
};
