/*============================================================================
==============================================================================
                      
                              initUserGraphics.c
 
==============================================================================
Remarks:

         Functions needed for user graphics
         simulation

============================================================================*/

#include "SL.h"
#include "SL_user.h"
#include "SL_man.h"

// openGL includes
#ifdef powerpc
#include <GLUT/glut.h>
#else
#include "GL/glut.h"
#endif
#include "SL_openGL.h"
#include "SL_userGraphics.h"

// global variables

// local variables

static void displayBall2(void *b);
static void displayBall1(void *b);


/*****************************************************************************
******************************************************************************
Function Name	: initUserGraphics
Date		: June 1999
   
Remarks:

      allows adding new graphics functions to openGL interface

******************************************************************************
Paramters:  (i/o = input/output)

  none   

*****************************************************************************/
int
initUserGraphics(void)

{
	addToUserGraphics("ball2","Display a 2cm diameter ball",displayBall2,(N_CART+1)*sizeof(double));

	addToUserGraphics("ball1","Display a 2cm diameter ball",displayBall1,(N_CART+1)*sizeof(double));


  return TRUE;

}


static void
displayBall2(void *b)
{
    GLfloat  col[4]={(float)1.0,(float)0.0,(float)0.0,(float)1.0};
    double    ball[N_CART+1];

    // assign the ball position from b array
    memcpy(&ball,b,(N_CART+1)*sizeof(double));

    /* here is the drawing rountines */
    glPushMatrix();
    glTranslated((GLdouble)ball[_X_],
                 (GLdouble)ball[_Y_],
                 (GLdouble)ball[_Z_]);

    glColor4fv(col);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, col);
    glutSolidSphere(0.02,8,8);
    glPopMatrix();
}

static void
displayBall1(void *b)
{
    GLfloat  col[4]={(float)1.0,(float)0.0,(float)1.0,(float)0.0};
    double    ball[N_CART+1];

    // assign the ball position from b array
    memcpy(&ball,b,(N_CART+1)*sizeof(double));

    /* here is the drawing rountines */
    glPushMatrix();
    glTranslated((GLdouble)ball[_X_],
                 (GLdouble)ball[_Y_],
                 (GLdouble)ball[_Z_]);

    glColor4fv(col);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, col);
    glutSolidSphere(0.02,8,8);
    glPopMatrix();

}
