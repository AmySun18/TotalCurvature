package curvature_total;


public class point2 {

    static final double infinitelyLarge = 99999;
    public point2() {}

    public point2(double X, double Y) {
        x = X;
        y = Y;
    }
    
    public point2(point2 p) {
        x = p.x;
        y = p.y;
    }

    double x=0.0, y=0.0;

    
    double Length() {
        return Math.sqrt(x * x + y * y);
    }

    point2 Norm1() {
        return new point2( -y, x);
    }

    point2 Norm2() {
        return new point2(y, -x);
    }

    static point2 ExtendToInfinite(point2 startPoint, point2 middlePoint) {
        return new point2(middlePoint.x +
                          infinitelyLarge * (middlePoint.x - startPoint.x),
                          middlePoint.y +
                          infinitelyLarge * (middlePoint.y - startPoint.y));
    }

    static point2 MiddlePoint(point2 a, point2 b)
    {
    	return new point2((a.x+b.x)/2,(a.y+b.y)/2);
    }
    
    static double Dist(point2 a, point2 b) {
       // point2 temp = new point2(b.x - a.x, b.y - a.y);
        return Math.sqrt((b.x - a.x )*( b.x - a.x) + (b.y - a.y) * (b.y - a.y));
    }

    void Normalize() {
        double foo = 1 / Length();
        x *= foo;
        y *= foo;
    }
    
    static point2 normalize(point2 a) {
        double foo = 1 / a.Length();
        return new point2(a.x*foo,a.y*foo);
    }

    static point2 plus(point2 a, point2  b)
{
        return new point2(a.x+b.x,a.y+b.y);
};


//returns point2-point2
static point2 minus(point2 a, point2 b)
{
        return new point2(a.x-b.x,a.y-b.y);
};


//returns point2*double
static point2 product(point2 a, double b)
{
        return new point2(a.x*b,a.y*b);
};

//returns point2/double
static point2 divide(point2 a, double b)
{
        return new point2(a.x/b,a.y/b);
};

// dot product
static double dot(point2  a, point2  b)
{
        return a.x * b.x + a.y * b.y;
};

//cross product
static double cross(point2  a, point2  b)
{
        return a.x*b.y-b.x*a.y;
}








}
