#include "polygon.h"
#include <celmath/mathlib.h>

using namespace std;

Polygon::Polygon
    (
        const Color &color,
        float opacity,
        const Color &shadowBoxColor
        )
    : m_updateDisplayList(true),
    m_color(color),
    m_opacity(opacity),
    m_shadowBoxColor(shadowBoxColor),
    m_displayList(0)
{
}


Polygon::~Polygon()
{
    // Destroy the display list
    glDeleteLists(m_displayList, 1);
}

void
Polygon::setColor(const Color &color)
{
    m_color=color;
    m_updateDisplayList=true;
}

void
Polygon::setOpacity(float opacity)
{
    m_opacity=opacity;
    m_updateDisplayList=true;
}

void
Polygon::render(bool drawShadowVolume) const
{
    if(m_updateDisplayList)
    {
        createDisplayList( drawShadowVolume ) ;
        m_updateDisplayList = false ;
    }

    // Render the display list (crash on intel chipset with old openGl implementation)
    glCallList(m_displayList);
}


void
Polygon::polygon2dtoPolygon3d(std::vector<Point2f>& geoPoly, float bodyRadius, Polygon& cartPoly)
{
    // Nothing to do
    if( geoPoly.empty() )
    {
        return ;
    }

    // Nettoyage du polygone de sortie
    cartPoly.clear() ;

    // Translate coordinates crossing date line
    int testCrossSide = 0 ;
    int firstCrossSide = 0 ;
    vector<Point2f>::iterator prev = geoPoly.begin() ;
    for( vector<Point2f>::iterator it2d = geoPoly.begin(); it2d!=geoPoly.end();++it2d)
    {
        // Detect first cross side
        if( firstCrossSide == 0 && crossDateLine( *prev, *it2d, &testCrossSide ) )
        {
            firstCrossSide = testCrossSide ;
        }

        // Cross EAST
        if( firstCrossSide > 0 && it2d->x <= 0 )
        {
            it2d->x += 360 ;
        }
        // Cross WEST
        else if( firstCrossSide < 0 && it2d->x >= 0 )
        {
            it2d->x -= 360 ;
        }

        // Update previous
        prev = it2d ;
    }


    // Compute the polygon area. If it's positive, then the polygon is clockwize.
    // Otherwise, we have to reverse it or it won't be visible
    float A = area( geoPoly )  ;
    bool clockwize ;

    if( A != 0 )
    {
        clockwize = A > 0 ;
    }
    else
    {
        // It's not a polygon in 2D, it's a line (poles)

        // A line is clockwize by comparing the line extremities, and the two poles works the opposite way
        clockwize = (geoPoly.front().x > geoPoly.back().x) && geoPoly.front().y < 0 ;
    }

    if( clockwize  )
    {
        vector<Point2f>::const_iterator it2d;
        for( vector<Point2f>::const_iterator it2d = geoPoly.begin(); it2d!=geoPoly.end();++it2d)
        {
            cartPoly.push_back(spherePointLL(it2d->x, it2d->y, bodyRadius));
        }
    }
    else
    {
        for( vector<Point2f>::const_reverse_iterator it2d = geoPoly.rbegin(); it2d!=geoPoly.rend();++it2d)
        {
            cartPoly.push_back(spherePointLL(it2d->x, it2d->y, bodyRadius));
        }
    }
}


float
Polygon::area( const std::vector<Point2f>& contour )
{
    int n = contour.size();

    float A = 0.0f;

    for(int p=n-1,q=0; q<n; p=q++)
    {
        A += contour[p].x*contour[q].y - contour[q].x*contour[p].y;
    }

    return A*0.5f;
}


static bool isIncreasing(const std::vector<Point2f>& contour) ;

Vector3d
Polygon::spherePointLL(float longi, float lati, float bodyRadius)
{
    double theta = celmath::degToRad(180 - longi);
    double phi = celmath::degToRad(lati);


    Vector3d point( cos(phi) * cos(theta),
                   sin(phi),
                   cos(phi) * sin(theta));
    point *= bodyRadius;
    return point;
}


bool
Polygon::crossDateLine(const Point2f &geoPt1, const Point2f &geoPt2, int *side)
{
    // Si distance de plus de 180 degr�s
    // Alors coupe la ligne de changement de date
    double delta = geoPt1.x - geoPt2.x;
    if ( delta > 180 && delta < 360 )
    {
        *side = 1 ;
        return true;
    }
    else if ( delta > -360 && delta < -180 )
    {
        *side = -1 ;
        return true;
    }
    *side = 0 ;
    return false ;
}


// Draws a polygon into a displaylist
void
Polygon::createDisplayList(bool drawShadowVolume)
    const
{
    // If a display list already exists
    if(m_displayList != 0)
    {
        // Destroy the display list
        glDeleteLists(m_displayList, 1);
    }

    // Generate the display list
    m_displayList = glGenLists(1);

    // Create the display list
    glNewList(m_displayList,GL_COMPILE);

    // Drawing
    Draw( drawShadowVolume ) ;

    // End of display list
    glEndList();
}


// Draws a polygon over the Earth using shadow volume technique
// Using Carmack's reverse method
// drawShadowVolume : draw filled volume
void
Polygon::Draw(bool drawShadowVolume)
    const
{
    // Disables any depth correction
    glDisable(GL_POLYGON_OFFSET_FILL);

    // Push attributes used for stencil rendering
    glPushAttrib( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT | GL_POLYGON_BIT | GL_STENCIL_BUFFER_BIT );

    // Clear stencil buffer and init value to 0
    glClearStencil(0);
    glClear(GL_STENCIL_BUFFER_BIT);

    // Turn off lighting
    glDisable( GL_LIGHTING );
    // Turn off texture
    glDisable(GL_TEXTURE_2D);
    // Turn off writing to the Depth-Buffer
    glDepthMask( GL_FALSE );
    glDepthFunc( GL_LEQUAL );

    // Turn On Stencil Buffer Testing
    glEnable( GL_STENCIL_TEST );

    // Don't Draw Into The Colour Buffer
    glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE );
    glStencilFunc( GL_ALWAYS, 1, 0xFFFFFFFFL );

    //Turn on Cull Face
    glEnable(GL_CULL_FACE);

    // Carmack's reverse
    // Render back face of shadow volume
    // If depth test fails, increment stencil value
    glFrontFace( GL_CW );
    glStencilOp( GL_KEEP , GL_INCR, GL_KEEP);
    DrawShadowVolume(*this);

    // Render front face of shadow volume.
    // If depth test fails, dkecrement stencil value
    glFrontFace( GL_CCW );
    glStencilOp( GL_KEEP , GL_DECR, GL_KEEP);
    DrawShadowVolume(*this);

    // Front faces rendered
    glFrontFace( GL_CCW );
    // Enable rendering to colour buffer
    glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );

    // Draw a shadowing rectangle covering the entire screen
    // Draw only when stencil value is equal to 1
    glStencilFunc( GL_EQUAL, 1, 0xFFFFFFFFL );
    //glStencilFunc( GL_NOTEQUAL, 0, 0xFFFFFFFFL );

    glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );
    DrawShadowColor();

    glPopAttrib();

    // Enable depth correction
    glEnable(GL_POLYGON_OFFSET_FILL);

    // Draw filled shadow volume
    if (drawShadowVolume)
    {
        auto shadowBoxColorV = m_shadowBoxColor.toVector3();
        glColor3f(shadowBoxColorV.x(), shadowBoxColorV.y(), shadowBoxColorV.z());
        glLineWidth(2);
        DrawShadowVolume(*this, false);
        glLineWidth(1);
    }

    glDisable(GL_STENCIL_TEST);
}


// Draw shadow pyramid
void Polygon::DrawShadowVolume(const vector<Vector3d> &polygon, bool filled) const
{
    Vector3d origin(0,0,0);

    if( polygon.size() == 0 )
    {
        return ;
    }

    // For untriangulated polygon
    vector<Vector3d>::const_iterator it;
    glBegin(filled ? GL_POLYGON : GL_LINE_LOOP);
    for (it = polygon.begin(); it != polygon.end(); ++it)
    {
        glVertex3dv(it->data());
    }
    glEnd();

    glBegin(filled ? GL_TRIANGLES : GL_LINE_LOOP);
    for (int i = 0; i < polygon.size() - 1; i++)
    {
        glVertex3dv(polygon[i].data());
        glVertex3dv(origin.data());
        glVertex3dv(polygon[i+1].data());
    }

    glVertex3dv(polygon[polygon.size() - 1].data());
    glVertex3dv(origin.data());
    glVertex3dv(polygon[0].data());
    glEnd();

}


void Polygon::DrawFarPlane() const
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glBegin(GL_QUADS);
    glVertex3d(1,1,1);
    glVertex3d(-1,1, 1);
    glVertex3d(-1,-1, 1);
    glVertex3d(1,-1, 1);
    glEnd();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}


// Draw Shadow color
// Draw a uniform color over all the screen
void Polygon::DrawShadowColor() const
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

//    glColor(m_color, m_opacity);
    glColor4f(m_color.red(), m_color.green(), m_color.blue(), m_opacity);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glBegin(GL_QUADS);
    glVertex2d(1,1);
    glVertex2d(-1,1);
    glVertex2d(-1,-1);
    glVertex2d(1,-1);
    glEnd();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}
