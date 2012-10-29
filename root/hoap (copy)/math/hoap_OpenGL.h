/* this function generates simple OpenGL graphics code to draw each link */


glPushMatrix();
myDrawGLElement((int)999,(double)0.0,(int)1);
glPopMatrix();

/* JointID = 0 */

glPushMatrix();
glPushMatrix();
if (basec[0].x[1]==0 && basec[0].x[2]==0)
glRotated((GLdouble)90.*(-1. + basec[0].x[3]/Sqrt(Power(basec[0].x[1],2) + Power(basec[0].x[2],2) + Power(basec[0].x[3],2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*basec[0].x[1],(GLdouble)0.5*basec[0].x[2],(GLdouble)0.5*(basec[0].x[3] + Sqrt(Power(basec[0].x[1],2) + Power(basec[0].x[2],2) + Power(basec[0].x[3],2))));
myDrawGLElement((int)0,(double)Sqrt(Power(basec[0].x[1],2) + Power(basec[0].x[2],2) + Power(basec[0].x[3],2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)basec[0].x[1],(GLdouble)basec[0].x[2],(GLdouble)basec[0].x[3]);
glRotated((GLdouble)114.59155902616465*ArcCos(baseo[0].q[1]),(GLdouble)baseo[0].q[2],(GLdouble)baseo[0].q[3],(GLdouble)baseo[0].q[4]);

/* JointID = 2102 */

glPushMatrix();
glPushMatrix();
if (0==0 && 0==0)
glRotated((GLdouble)90.*(-1. + BODYLINK1/Sqrt(Power(BODYLINK1,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.,(GLdouble)0.,(GLdouble)0.5*(BODYLINK1 + Sqrt(Power(BODYLINK1,2))));
myDrawGLElement((int)2102,(double)Sqrt(Power(BODYLINK1,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)0,(GLdouble)BODYLINK1);
glRotated((GLdouble)0.,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 7 */

glPushMatrix();
glPushMatrix();
if (0==0 && -ARMLINK1==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.,(GLdouble)-0.5*ARMLINK1,(GLdouble)0.5*Sqrt(Power(ARMLINK1,2)));
myDrawGLElement((int)7,(double)Sqrt(Power(ARMLINK1,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)-ARMLINK1,(GLdouble)0);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[7].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 8 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)8,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[8].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 9 */

glPushMatrix();
glPushMatrix();
if (0==0 && -ARMLINK2==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.,(GLdouble)-0.5*ARMLINK2,(GLdouble)0.5*Sqrt(Power(ARMLINK2,2)));
myDrawGLElement((int)9,(double)Sqrt(Power(ARMLINK2,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)-ARMLINK2,(GLdouble)0);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[9].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 10 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)10,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[10].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 25 */

glPushMatrix();
glPushMatrix();
if (0==0 && ARMLINK3==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.,(GLdouble)0.5*ARMLINK3,(GLdouble)0.5*Sqrt(Power(ARMLINK3,2)));
myDrawGLElement((int)25,(double)Sqrt(Power(ARMLINK3,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)ARMLINK3,(GLdouble)0);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0.,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 26 */

glPushMatrix();
glPushMatrix();
if (GRIPJOFFX==0 && 0==0)
glRotated((GLdouble)90.*(-1. + GRIPJOFFZ/Sqrt(Power(GRIPJOFFX,2) + Power(GRIPJOFFZ,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*GRIPJOFFX,(GLdouble)0.,(GLdouble)0.5*(GRIPJOFFZ + Sqrt(Power(GRIPJOFFX,2) + Power(GRIPJOFFZ,2))));
myDrawGLElement((int)26,(double)Sqrt(Power(GRIPJOFFX,2) + Power(GRIPJOFFZ,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)GRIPJOFFX,(GLdouble)0,(GLdouble)GRIPJOFFZ);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)0.,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 2601 */

glPushMatrix();
glPushMatrix();
if (FINGER==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*FINGER,(GLdouble)0.,(GLdouble)0.5*Sqrt(Power(FINGER,2)));
myDrawGLElement((int)2601,(double)Sqrt(Power(FINGER,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 17 */

glPushMatrix();
glPushMatrix();
if (0==0 && ARMLINK1==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.,(GLdouble)0.5*ARMLINK1,(GLdouble)0.5*Sqrt(Power(ARMLINK1,2)));
myDrawGLElement((int)17,(double)Sqrt(Power(ARMLINK1,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)ARMLINK1,(GLdouble)0);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[17].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 18 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)18,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[18].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 19 */

glPushMatrix();
glPushMatrix();
if (0==0 && -ARMLINK2==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.,(GLdouble)-0.5*ARMLINK2,(GLdouble)0.5*Sqrt(Power(ARMLINK2,2)));
myDrawGLElement((int)19,(double)Sqrt(Power(ARMLINK2,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)-ARMLINK2,(GLdouble)0);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[19].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 20 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)20,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[20].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 27 */

glPushMatrix();
glPushMatrix();
if (0==0 && ARMLINK3==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.,(GLdouble)0.5*ARMLINK3,(GLdouble)0.5*Sqrt(Power(ARMLINK3,2)));
myDrawGLElement((int)27,(double)Sqrt(Power(ARMLINK3,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)0,(GLdouble)ARMLINK3,(GLdouble)0);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)-90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[27].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 28 */

glPushMatrix();
glPushMatrix();
if (-GRIPJOFFX==0 && 0==0)
glRotated((GLdouble)90.*(-1. + GRIPJOFFZ/Sqrt(Power(GRIPJOFFX,2) + Power(GRIPJOFFZ,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*GRIPJOFFX,(GLdouble)0.,(GLdouble)0.5*(GRIPJOFFZ + Sqrt(Power(GRIPJOFFX,2) + Power(GRIPJOFFZ,2))));
myDrawGLElement((int)28,(double)Sqrt(Power(GRIPJOFFX,2) + Power(GRIPJOFFZ,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-GRIPJOFFX,(GLdouble)0,(GLdouble)GRIPJOFFZ);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[28].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 2801 */

glPushMatrix();
glPushMatrix();
if (FINGER==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*FINGER,(GLdouble)0.,(GLdouble)0.5*Sqrt(Power(FINGER,2)));
myDrawGLElement((int)2801,(double)Sqrt(Power(FINGER,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 2701 */

glPushMatrix();
glPushMatrix();
if (-PALMOFFSETX==0 && 0==0)
glRotated((GLdouble)90.*(-1. + PALM/Sqrt(Power(PALM,2) + Power(PALMOFFSETX,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*PALMOFFSETX,(GLdouble)0.,(GLdouble)0.5*(PALM + Sqrt(Power(PALM,2) + Power(PALMOFFSETX,2))));
myDrawGLElement((int)2701,(double)Sqrt(Power(PALM,2) + Power(PALMOFFSETX,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-PALMOFFSETX,(GLdouble)0,(GLdouble)PALM);
glRotated((GLdouble)0.,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 2700 */

glPushMatrix();
glPushMatrix();
if (eff[4].x[1]==0 && eff[4].x[2]==0)
glRotated((GLdouble)90.*(-1. + eff[4].x[3]/Sqrt(Power(eff[4].x[1],2) + Power(eff[4].x[2],2) + Power(eff[4].x[3],2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*eff[4].x[1],(GLdouble)0.5*eff[4].x[2],(GLdouble)0.5*(eff[4].x[3] + Sqrt(Power(eff[4].x[1],2) + Power(eff[4].x[2],2) + Power(eff[4].x[3],2))));
myDrawGLElement((int)2700,(double)Sqrt(Power(eff[4].x[1],2) + Power(eff[4].x[2],2) + Power(eff[4].x[3],2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 22 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)22,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)57.29577951308232*state[22].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 23 */

glPushMatrix();
glPushMatrix();
if (HEADLINK2==0 && 0==0)
glRotated((GLdouble)90.*(-1. + HEADLINK1/Sqrt(Power(HEADLINK1,2) + Power(HEADLINK2,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*HEADLINK2,(GLdouble)0.,(GLdouble)0.5*(HEADLINK1 + Sqrt(Power(HEADLINK1,2) + Power(HEADLINK2,2))));
myDrawGLElement((int)23,(double)Sqrt(Power(HEADLINK1,2) + Power(HEADLINK2,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)HEADLINK2,(GLdouble)0,(GLdouble)HEADLINK1);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)0.,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 24 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)24,(double)0,(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 21 */

glPushMatrix();
glPushMatrix();
if (BODYLINK2==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*BODYLINK2,(GLdouble)0.,(GLdouble)0.5*Sqrt(Power(BODYLINK2,2)));
myDrawGLElement((int)21,(double)Sqrt(Power(BODYLINK2,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)BODYLINK2,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[21].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 1 */

glPushMatrix();
glPushMatrix();
if (-WAISTLINK2==0 && -WAISTLINK1==0)
glRotated((GLdouble)90.*(-1. + LEGLINK1/Sqrt(Power(LEGLINK1,2) + Power(WAISTLINK1,2) + Power(WAISTLINK2,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*WAISTLINK2,(GLdouble)-0.5*WAISTLINK1,(GLdouble)0.5*(LEGLINK1 + Sqrt(Power(LEGLINK1,2) + Power(WAISTLINK1,2) + Power(WAISTLINK2,2))));
myDrawGLElement((int)1,(double)Sqrt(Power(LEGLINK1,2) + Power(WAISTLINK1,2) + Power(WAISTLINK2,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-WAISTLINK2,(GLdouble)-WAISTLINK1,(GLdouble)LEGLINK1);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[1].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 2 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)2,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[2].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 3 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)3,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[3].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 4 */

glPushMatrix();
glPushMatrix();
if (-LEGLINK2==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*LEGLINK2,(GLdouble)0.,(GLdouble)0.5*Sqrt(Power(LEGLINK2,2)));
myDrawGLElement((int)4,(double)Sqrt(Power(LEGLINK2,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-LEGLINK2,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[4].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 5 */

glPushMatrix();
glPushMatrix();
if (-LEGLINK3==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*LEGLINK3,(GLdouble)0.,(GLdouble)0.5*Sqrt(Power(LEGLINK3,2)));
myDrawGLElement((int)5,(double)Sqrt(Power(LEGLINK3,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-LEGLINK3,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[5].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 6 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)6,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[6].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 601 */

glPushMatrix();
glPushMatrix();
if (-LEGLINK4==0 && FOOTLLENGLONG==0)
glRotated((GLdouble)90.*(-1. + FOOTWIDLONG/Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*LEGLINK4,(GLdouble)0.5*FOOTLLENGLONG,(GLdouble)0.5*(FOOTWIDLONG + Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2))));
myDrawGLElement((int)601,(double)Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 602 */

glPushMatrix();
glPushMatrix();
if (-LEGLINK4==0 && FOOTLLENGLONG==0)
glRotated((GLdouble)90.*(-1. - FOOTWIDSHORT/Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*LEGLINK4,(GLdouble)0.5*FOOTLLENGLONG,(GLdouble)0.5*(-FOOTWIDSHORT + Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2))));
myDrawGLElement((int)602,(double)Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 603 */

glPushMatrix();
glPushMatrix();
if (-LEGLINK4==0 && -FOOTLENGSHORT==0)
glRotated((GLdouble)90.*(-1. - FOOTWIDSHORT/Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*LEGLINK4,(GLdouble)-0.5*FOOTLENGSHORT,(GLdouble)0.5*(-FOOTWIDSHORT + Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2))));
myDrawGLElement((int)603,(double)Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 604 */

glPushMatrix();
glPushMatrix();
if (-LEGLINK4==0 && -FOOTLENGSHORT==0)
glRotated((GLdouble)90.*(-1. + FOOTWIDLONG/Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*LEGLINK4,(GLdouble)-0.5*FOOTLENGSHORT,(GLdouble)0.5*(FOOTWIDLONG + Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2))));
myDrawGLElement((int)604,(double)Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 600 */

glPushMatrix();
glPushMatrix();
if (eff[1].x[1]==0 && eff[1].x[2]==0)
glRotated((GLdouble)90.*(-1. + eff[1].x[3]/Sqrt(Power(eff[1].x[1],2) + Power(eff[1].x[2],2) + Power(eff[1].x[3],2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*eff[1].x[1],(GLdouble)0.5*eff[1].x[2],(GLdouble)0.5*(eff[1].x[3] + Sqrt(Power(eff[1].x[1],2) + Power(eff[1].x[2],2) + Power(eff[1].x[3],2))));
myDrawGLElement((int)600,(double)Sqrt(Power(eff[1].x[1],2) + Power(eff[1].x[2],2) + Power(eff[1].x[3],2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 11 */

glPushMatrix();
glPushMatrix();
if (-WAISTLINK2==0 && -WAISTLINK1==0)
glRotated((GLdouble)90.*(-1. - LEGLINK1/Sqrt(Power(LEGLINK1,2) + Power(WAISTLINK1,2) + Power(WAISTLINK2,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*WAISTLINK2,(GLdouble)-0.5*WAISTLINK1,(GLdouble)0.5*(-LEGLINK1 + Sqrt(Power(LEGLINK1,2) + Power(WAISTLINK1,2) + Power(WAISTLINK2,2))));
myDrawGLElement((int)11,(double)Sqrt(Power(LEGLINK1,2) + Power(WAISTLINK1,2) + Power(WAISTLINK2,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-WAISTLINK2,(GLdouble)-WAISTLINK1,(GLdouble)-LEGLINK1);
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[11].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 12 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)12,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)57.29577951308232*state[12].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 13 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)13,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[13].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 14 */

glPushMatrix();
glPushMatrix();
if (-LEGLINK2==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*LEGLINK2,(GLdouble)0.,(GLdouble)0.5*Sqrt(Power(LEGLINK2,2)));
myDrawGLElement((int)14,(double)Sqrt(Power(LEGLINK2,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-LEGLINK2,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)57.29577951308232*state[14].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 15 */

glPushMatrix();
glPushMatrix();
if (-LEGLINK3==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*LEGLINK3,(GLdouble)0.,(GLdouble)0.5*(0. + Sqrt(0. + Power(LEGLINK3,2))));
myDrawGLElement((int)15,(double)Sqrt(0. + Power(LEGLINK3,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-LEGLINK3,(GLdouble)0,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[15].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 16 */

glPushMatrix();
glPushMatrix();
myDrawGLElement((int)16,(double)0,(int)0);
glPopMatrix();
glRotated((GLdouble)-90.,(GLdouble)1.,(GLdouble)0.,(GLdouble)0.);
glRotated((GLdouble)57.29577951308232*state[16].th,(GLdouble)0,(GLdouble)0,(GLdouble)1);

/* JointID = 1601 */

glPushMatrix();
glPushMatrix();
if (-LEGLINK4==0 && 0==0)
glRotated((GLdouble)-90.,(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*LEGLINK4,(GLdouble)0.,(GLdouble)0.5*Sqrt(Power(LEGLINK4,2)));
myDrawGLElement((int)1601,(double)Sqrt(Power(LEGLINK4,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-LEGLINK4,(GLdouble)0,(GLdouble)0);
glRotated((GLdouble)90.,(GLdouble)0.,(GLdouble)1.,(GLdouble)0.);
glRotated((GLdouble)180.,(GLdouble)0.,(GLdouble)0.,(GLdouble)1.);
glRotated((GLdouble)0.,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 1600 */

glPushMatrix();
glPushMatrix();
if (eff[3].x[1]==0 && eff[3].x[2]==0)
glRotated((GLdouble)90.*(-1. + eff[3].x[3]/Sqrt(Power(eff[3].x[1],2) + Power(eff[3].x[2],2) + Power(eff[3].x[3],2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*eff[3].x[1],(GLdouble)0.5*eff[3].x[2],(GLdouble)0.5*(eff[3].x[3] + Sqrt(Power(eff[3].x[1],2) + Power(eff[3].x[2],2) + Power(eff[3].x[3],2))));
myDrawGLElement((int)1600,(double)Sqrt(Power(eff[3].x[1],2) + Power(eff[3].x[2],2) + Power(eff[3].x[3],2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 1602 */

glPushMatrix();
glPushMatrix();
if (FOOTLLENGLONG==0 && FOOTWIDLONG==0)
glRotated((GLdouble)90.*(-1. - LEGLINK4/Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*FOOTLLENGLONG,(GLdouble)0.5*FOOTWIDLONG,(GLdouble)0.5*(-LEGLINK4 + Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2))));
myDrawGLElement((int)1602,(double)Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 1603 */

glPushMatrix();
glPushMatrix();
if (FOOTLLENGLONG==0 && -FOOTWIDSHORT==0)
glRotated((GLdouble)90.*(-1. - LEGLINK4/Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*FOOTLLENGLONG,(GLdouble)-0.5*FOOTWIDSHORT,(GLdouble)0.5*(-LEGLINK4 + Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2))));
myDrawGLElement((int)1603,(double)Sqrt(Power(FOOTLLENGLONG,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 1604 */

glPushMatrix();
glPushMatrix();
if (-FOOTLENGSHORT==0 && -FOOTWIDSHORT==0)
glRotated((GLdouble)90.*(-1. - LEGLINK4/Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*FOOTLENGSHORT,(GLdouble)-0.5*FOOTWIDSHORT,(GLdouble)0.5*(-LEGLINK4 + Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2))));
myDrawGLElement((int)1604,(double)Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDSHORT,2) + Power(LEGLINK4,2)),(int)0);
glPopMatrix();
glPopMatrix();

/* JointID = 1605 */

glPushMatrix();
glPushMatrix();
if (-FOOTLENGSHORT==0 && FOOTWIDLONG==0)
glRotated((GLdouble)90.*(-1. - LEGLINK4/Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*FOOTLENGSHORT,(GLdouble)0.5*FOOTWIDLONG,(GLdouble)0.5*(-LEGLINK4 + Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2))));
myDrawGLElement((int)1605,(double)Sqrt(Power(FOOTLENGSHORT,2) + Power(FOOTWIDLONG,2) + Power(LEGLINK4,2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();

/* JointID = 2501 */

glPushMatrix();
glPushMatrix();
if (-PALMOFFSETX==0 && 0==0)
glRotated((GLdouble)90.*(-1. + PALM/Sqrt(Power(PALM,2) + Power(PALMOFFSETX,2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)-0.5*PALMOFFSETX,(GLdouble)0.,(GLdouble)0.5*(PALM + Sqrt(Power(PALM,2) + Power(PALMOFFSETX,2))));
myDrawGLElement((int)2501,(double)Sqrt(Power(PALM,2) + Power(PALMOFFSETX,2)),(int)1);
glPopMatrix();
glTranslated((GLdouble)-PALMOFFSETX,(GLdouble)0,(GLdouble)PALM);
glRotated((GLdouble)0.,(GLdouble)0,(GLdouble)0,(GLdouble)0);

/* JointID = 2500 */

glPushMatrix();
glPushMatrix();
if (eff[2].x[1]==0 && eff[2].x[2]==0)
glRotated((GLdouble)90.*(-1. + eff[2].x[3]/Sqrt(Power(eff[2].x[1],2) + Power(eff[2].x[2],2) + Power(eff[2].x[3],2))),(GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
else
glRotated((GLdouble)180.0,(GLdouble)0.5*eff[2].x[1],(GLdouble)0.5*eff[2].x[2],(GLdouble)0.5*(eff[2].x[3] + Sqrt(Power(eff[2].x[1],2) + Power(eff[2].x[2],2) + Power(eff[2].x[3],2))));
myDrawGLElement((int)2500,(double)Sqrt(Power(eff[2].x[1],2) + Power(eff[2].x[2],2) + Power(eff[2].x[3],2)),(int)0);
glPopMatrix();
glPopMatrix();
glPopMatrix();
glPopMatrix();
/*glutSwapBuffers();*/
