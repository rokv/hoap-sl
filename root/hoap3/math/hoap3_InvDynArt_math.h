/* sine and cosine precomputation */
sstate22th=Sin(state[22].th);
cstate22th=Cos(state[22].th);

sstate23th=Sin(state[23].th);
cstate23th=Cos(state[23].th);

sstate24th=Sin(state[24].th);
cstate24th=Cos(state[24].th);

sstate7th=Sin(state[7].th);
cstate7th=Cos(state[7].th);

sstate8th=Sin(state[8].th);
cstate8th=Cos(state[8].th);

sstate9th=Sin(state[9].th);
cstate9th=Cos(state[9].th);

sstate10th=Sin(state[10].th);
cstate10th=Cos(state[10].th);

sstate17th=Sin(state[17].th);
cstate17th=Cos(state[17].th);

sstate18th=Sin(state[18].th);
cstate18th=Cos(state[18].th);

sstate19th=Sin(state[19].th);
cstate19th=Cos(state[19].th);

sstate20th=Sin(state[20].th);
cstate20th=Cos(state[20].th);

sstate21th=Sin(state[21].th);
cstate21th=Cos(state[21].th);

sstate1th=Sin(state[1].th);
cstate1th=Cos(state[1].th);

sstate2th=Sin(state[2].th);
cstate2th=Cos(state[2].th);

sstate3th=Sin(state[3].th);
cstate3th=Cos(state[3].th);

sstate4th=Sin(state[4].th);
cstate4th=Cos(state[4].th);

sstate5th=Sin(state[5].th);
cstate5th=Cos(state[5].th);

sstate6th=Sin(state[6].th);
cstate6th=Cos(state[6].th);

sstate11th=Sin(state[11].th);
cstate11th=Cos(state[11].th);

sstate12th=Sin(state[12].th);
cstate12th=Cos(state[12].th);

sstate13th=Sin(state[13].th);
cstate13th=Cos(state[13].th);

sstate14th=Sin(state[14].th);
cstate14th=Cos(state[14].th);

sstate15th=Sin(state[15].th);
cstate15th=Cos(state[15].th);

sstate16th=Sin(state[16].th);
cstate16th=Cos(state[16].th);


/* rotation matrix sine and cosine precomputation */

























rseff1a1=Sin(eff[1].a[1]);
rceff1a1=Cos(eff[1].a[1]);

rseff1a2=Sin(eff[1].a[2]);
rceff1a2=Cos(eff[1].a[2]);

rseff1a3=Sin(eff[1].a[3]);
rceff1a3=Cos(eff[1].a[3]);












rseff2a1=Sin(eff[2].a[1]);
rceff2a1=Cos(eff[2].a[1]);

rseff2a2=Sin(eff[2].a[2]);
rceff2a2=Cos(eff[2].a[2]);

rseff2a3=Sin(eff[2].a[3]);
rceff2a3=Cos(eff[2].a[3]);



hoap3_InvDynArtfunc1();

hoap3_InvDynArtfunc2();

hoap3_InvDynArtfunc3();

hoap3_InvDynArtfunc4();

hoap3_InvDynArtfunc5();

hoap3_InvDynArtfunc6();

hoap3_InvDynArtfunc7();

hoap3_InvDynArtfunc8();

hoap3_InvDynArtfunc9();

hoap3_InvDynArtfunc10();

hoap3_InvDynArtfunc11();

hoap3_InvDynArtfunc12();

hoap3_InvDynArtfunc13();

hoap3_InvDynArtfunc14();

hoap3_InvDynArtfunc15();

hoap3_InvDynArtfunc16();

hoap3_InvDynArtfunc17();

hoap3_InvDynArtfunc18();

hoap3_InvDynArtfunc19();

hoap3_InvDynArtfunc20();

hoap3_InvDynArtfunc21();

hoap3_InvDynArtfunc22();

hoap3_InvDynArtfunc23();

hoap3_InvDynArtfunc24();

hoap3_InvDynArtfunc25();

hoap3_InvDynArtfunc26();

hoap3_InvDynArtfunc27();

hoap3_InvDynArtfunc28();

hoap3_InvDynArtfunc29();

hoap3_InvDynArtfunc30();

hoap3_InvDynArtfunc31();

hoap3_InvDynArtfunc32();

hoap3_InvDynArtfunc33();

hoap3_InvDynArtfunc34();

hoap3_InvDynArtfunc35();

hoap3_InvDynArtfunc36();

hoap3_InvDynArtfunc37();

hoap3_InvDynArtfunc38();

hoap3_InvDynArtfunc39();

hoap3_InvDynArtfunc40();

hoap3_InvDynArtfunc41();

hoap3_InvDynArtfunc42();

hoap3_InvDynArtfunc43();

hoap3_InvDynArtfunc44();

hoap3_InvDynArtfunc45();

/* acceleration vectors, base acceleration, and joint torques */
a2[1]=c2[1];
a2[2]=c2[2];
a2[3]=state[22].thdd;
a2[4]=c2[4];
a2[5]=c2[5];

a3[1]=c3[1] + a2[1]*S32[1][1] + a2[3]*S32[1][3];
a3[2]=c3[2] + a2[1]*S32[2][1] + a2[3]*S32[2][3];
a3[3]=state[23].thdd + a2[2];
a3[4]=c3[4] + a2[4]*S32[1][1] + a2[2]*(HEADLINK1*S32[1][1] - HEADLINK2*S32[1][3]);
a3[5]=c3[5] + a2[4]*S32[2][1] + a2[2]*(HEADLINK1*S32[2][1] - HEADLINK2*S32[2][3]);
a3[6]=-(HEADLINK1*a2[1]) + HEADLINK2*a2[3] + a2[5];

a4[1]=state[24].thdd + a3[1];
a4[2]=c4[2] + a3[2]*S43[2][2] + a3[3]*S43[2][3];
a4[3]=c4[3] + a3[2]*S43[3][2] + a3[3]*S43[3][3];
a4[4]=a3[4];
a4[5]=c4[5] + a3[5]*S43[2][2] + a3[6]*S43[2][3];
a4[6]=c4[6] + a3[5]*S43[3][2] + a3[6]*S43[3][3];

a5[1]=c5[1];
a5[2]=c5[2];
a5[3]=state[7].thdd;
a5[4]=c5[4];
a5[5]=c5[5];

a6[1]=c6[1] + a5[1]*S65[1][1] + a5[3]*S65[1][3];
a6[2]=c6[2] + a5[1]*S65[2][1] + a5[3]*S65[2][3];
a6[3]=state[8].thdd - a5[2];
a6[4]=c6[4] + a5[4]*S65[1][1];
a6[5]=c6[5] + a5[4]*S65[2][1];
a6[6]=-a5[5];

a7[1]=state[9].thdd + a6[3];
a7[2]=c7[2] + a6[1]*S76[2][1] + a6[2]*S76[2][2];
a7[3]=c7[3] + a6[1]*S76[3][1] + a6[2]*S76[3][2];
a7[4]=-(ARMLINK2*a6[1]) + a6[6];
a7[5]=c7[5] + ARMLINK2*a6[3]*S76[2][1] + a6[4]*S76[2][1] + a6[5]*S76[2][2];
a7[6]=c7[6] + ARMLINK2*a6[3]*S76[3][1] + a6[4]*S76[3][1] + a6[5]*S76[3][2];

a8[1]=c8[1] + a7[1]*S87[1][1] + a7[3]*S87[1][3];
a8[2]=c8[2] + a7[1]*S87[2][1] + a7[3]*S87[2][3];
a8[3]=state[10].thdd - a7[2];
a8[4]=c8[4] + a7[4]*S87[1][1] + a7[6]*S87[1][3];
a8[5]=c8[5] + a7[4]*S87[2][1] + a7[6]*S87[2][3];
a8[6]=-a7[5];

a9[1]=a8[1];
a9[2]=a8[2];
a9[3]=a8[3];
a9[4]=-(ARMLINK3*a8[3]) + a8[4];
a9[5]=a8[5];
a9[6]=ARMLINK3*a8[1] + a8[6];

a10[1]=c10[1];
a10[2]=c10[2];
a10[3]=state[17].thdd;
a10[4]=c10[4];
a10[5]=c10[5];

a11[1]=c11[1] + a10[1]*S1110[1][1] + a10[3]*S1110[1][3];
a11[2]=c11[2] + a10[1]*S1110[2][1] + a10[3]*S1110[2][3];
a11[3]=state[18].thdd - a10[2];
a11[4]=c11[4] + a10[4]*S1110[1][1];
a11[5]=c11[5] + a10[4]*S1110[2][1];
a11[6]=-a10[5];

a12[1]=state[19].thdd + a11[3];
a12[2]=c12[2] + a11[1]*S1211[2][1] + a11[2]*S1211[2][2];
a12[3]=c12[3] + a11[1]*S1211[3][1] + a11[2]*S1211[3][2];
a12[4]=-(ARMLINK2*a11[1]) + a11[6];
a12[5]=c12[5] + ARMLINK2*a11[3]*S1211[2][1] + a11[4]*S1211[2][1] + a11[5]*S1211[2][2];
a12[6]=c12[6] + ARMLINK2*a11[3]*S1211[3][1] + a11[4]*S1211[3][1] + a11[5]*S1211[3][2];

a13[1]=c13[1] + a12[1]*S1312[1][1] + a12[3]*S1312[1][3];
a13[2]=c13[2] + a12[1]*S1312[2][1] + a12[3]*S1312[2][3];
a13[3]=state[20].thdd - a12[2];
a13[4]=c13[4] + a12[4]*S1312[1][1] + a12[6]*S1312[1][3];
a13[5]=c13[5] + a12[4]*S1312[2][1] + a12[6]*S1312[2][3];
a13[6]=-a12[5];

a14[1]=a13[1];
a14[2]=a13[2];
a14[3]=a13[3];
a14[4]=-(ARMLINK3*a13[3]) + a13[4];
a14[5]=a13[5];
a14[6]=ARMLINK3*a13[1] + a13[6];

a15[1]=c15[1];
a15[2]=c15[2];
a15[3]=state[21].thdd;
a15[4]=c15[4];
a15[5]=c15[5];

a16[1]=c16[1] + a15[1]*S1615[1][1] + a15[3]*S1615[1][3];
a16[2]=c16[2] + a15[1]*S1615[2][1] + a15[3]*S1615[2][3];
a16[3]=state[1].thdd + a15[2];
a16[4]=c16[4] + WAISTLINK1*a15[3]*S1615[1][1] + a15[4]*S1615[1][1] - WAISTLINK1*a15[1]*S1615[1][3] + a15[2]*(LEGLINK1*S1615[1][1] + WAISTLINK2*S1615[1][3]);
a16[5]=c16[5] + WAISTLINK1*a15[3]*S1615[2][1] + a15[4]*S1615[2][1] - WAISTLINK1*a15[1]*S1615[2][3] + a15[2]*(LEGLINK1*S1615[2][1] + WAISTLINK2*S1615[2][3]);
a16[6]=-(LEGLINK1*a15[1]) - WAISTLINK2*a15[3] + a15[5];

a17[1]=c17[1] + a16[1]*S1716[1][1] + a16[3]*S1716[1][3];
a17[2]=c17[2] + a16[1]*S1716[2][1] + a16[3]*S1716[2][3];
a17[3]=state[2].thdd - a16[2];
a17[4]=c17[4] + a16[4]*S1716[1][1] + a16[6]*S1716[1][3];
a17[5]=c17[5] + a16[4]*S1716[2][1] + a16[6]*S1716[2][3];
a17[6]=-a16[5];

a18[1]=c18[1] + a17[1]*S1817[1][1] + a17[3]*S1817[1][3];
a18[2]=c18[2] + a17[1]*S1817[2][1] + a17[3]*S1817[2][3];
a18[3]=state[3].thdd - a17[2];
a18[4]=c18[4] + a17[4]*S1817[1][1] + a17[6]*S1817[1][3];
a18[5]=c18[5] + a17[4]*S1817[2][1] + a17[6]*S1817[2][3];
a18[6]=-a17[5];

a19[1]=c19[1] + a18[1]*S1918[1][1] + a18[2]*S1918[1][2];
a19[2]=c19[2] + a18[1]*S1918[2][1] + a18[2]*S1918[2][2];
a19[3]=state[4].thdd + a18[3];
a19[4]=c19[4] + a18[4]*S1918[1][1] - LEGLINK2*a18[3]*S1918[1][2] + a18[5]*S1918[1][2];
a19[5]=c19[5] + a18[4]*S1918[2][1] - LEGLINK2*a18[3]*S1918[2][2] + a18[5]*S1918[2][2];
a19[6]=LEGLINK2*a18[2] + a18[6];

a20[1]=c20[1] + a19[1]*S2019[1][1] + a19[2]*S2019[1][2];
a20[2]=c20[2] + a19[1]*S2019[2][1] + a19[2]*S2019[2][2];
a20[3]=state[5].thdd + a19[3];
a20[4]=c20[4] + a19[4]*S2019[1][1] - LEGLINK3*a19[3]*S2019[1][2] + a19[5]*S2019[1][2];
a20[5]=c20[5] + a19[4]*S2019[2][1] - LEGLINK3*a19[3]*S2019[2][2] + a19[5]*S2019[2][2];
a20[6]=LEGLINK3*a19[2] + a19[6];

a21[1]=c21[1] + a20[1]*S2120[1][1] + a20[3]*S2120[1][3];
a21[2]=c21[2] + a20[1]*S2120[2][1] + a20[3]*S2120[2][3];
a21[3]=state[6].thdd + a20[2];
a21[4]=c21[4] + a20[4]*S2120[1][1] + a20[6]*S2120[1][3];
a21[5]=c21[5] + a20[4]*S2120[2][1] + a20[6]*S2120[2][3];
a21[6]=a20[5];

a22[1]=a21[1];
a22[2]=a21[2];
a22[3]=a21[3];
a22[4]=FOOTLLENGLONG*a21[2] - FOOTWIDLONG*a21[3] + a21[4];
a22[5]=-(FOOTLLENGLONG*a21[1]) - LEGLINK4*a21[3] + a21[5];
a22[6]=FOOTWIDLONG*a21[1] + LEGLINK4*a21[2] + a21[6];

a23[1]=a21[1];
a23[2]=a21[2];
a23[3]=a21[3];
a23[4]=FOOTLLENGLONG*a21[2] + FOOTWIDSHORT*a21[3] + a21[4];
a23[5]=-(FOOTLLENGLONG*a21[1]) - LEGLINK4*a21[3] + a21[5];
a23[6]=-(FOOTWIDSHORT*a21[1]) + LEGLINK4*a21[2] + a21[6];

a24[1]=a21[1];
a24[2]=a21[2];
a24[3]=a21[3];
a24[4]=-(FOOTLENGSHORT*a21[2]) + FOOTWIDSHORT*a21[3] + a21[4];
a24[5]=FOOTLENGSHORT*a21[1] - LEGLINK4*a21[3] + a21[5];
a24[6]=-(FOOTWIDSHORT*a21[1]) + LEGLINK4*a21[2] + a21[6];

a25[1]=a21[1];
a25[2]=a21[2];
a25[3]=a21[3];
a25[4]=-(FOOTLENGSHORT*a21[2]) - FOOTWIDLONG*a21[3] + a21[4];
a25[5]=FOOTLENGSHORT*a21[1] - LEGLINK4*a21[3] + a21[5];
a25[6]=FOOTWIDLONG*a21[1] + LEGLINK4*a21[2] + a21[6];

a26[1]=a21[1]*S2621[1][1] + a21[2]*S2621[1][2] + a21[3]*S2621[1][3];
a26[2]=a21[1]*S2621[2][1] + a21[2]*S2621[2][2] + a21[3]*S2621[2][3];
a26[3]=a21[1]*S2621[3][1] + a21[2]*S2621[3][2] + a21[3]*S2621[3][3];
a26[4]=a21[4]*S2621[1][1] + a21[5]*S2621[1][2] + a21[3]*(-(eff[1].x[2]*S2621[1][1]) + eff[1].x[1]*S2621[1][2]) + a21[6]*S2621[1][3] + a21[2]*(eff[1].x[3]*S2621[1][1] - eff[1].x[1]*S2621[1][3]) + a21[1]*(-(eff[1].x[3]*S2621[1][2]) + eff[1].x[2]*S2621[1][3]);
a26[5]=a21[4]*S2621[2][1] + a21[5]*S2621[2][2] + a21[3]*(-(eff[1].x[2]*S2621[2][1]) + eff[1].x[1]*S2621[2][2]) + a21[6]*S2621[2][3] + a21[2]*(eff[1].x[3]*S2621[2][1] - eff[1].x[1]*S2621[2][3]) + a21[1]*(-(eff[1].x[3]*S2621[2][2]) + eff[1].x[2]*S2621[2][3]);
a26[6]=a21[4]*S2621[3][1] + a21[5]*S2621[3][2] + a21[3]*(-(eff[1].x[2]*S2621[3][1]) + eff[1].x[1]*S2621[3][2]) + a21[6]*S2621[3][3] + a21[2]*(eff[1].x[3]*S2621[3][1] - eff[1].x[1]*S2621[3][3]) + a21[1]*(-(eff[1].x[3]*S2621[3][2]) + eff[1].x[2]*S2621[3][3]);

a27[1]=c27[1] + a15[1]*S2715[1][1] + a15[3]*S2715[1][3];
a27[2]=c27[2] + a15[1]*S2715[2][1] + a15[3]*S2715[2][3];
a27[3]=state[11].thdd + a15[2];
a27[4]=c27[4] + WAISTLINK1*a15[3]*S2715[1][1] + a15[4]*S2715[1][1] - WAISTLINK1*a15[1]*S2715[1][3] + a15[2]*(-(LEGLINK1*S2715[1][1]) + WAISTLINK2*S2715[1][3]);
a27[5]=c27[5] + WAISTLINK1*a15[3]*S2715[2][1] + a15[4]*S2715[2][1] - WAISTLINK1*a15[1]*S2715[2][3] + a15[2]*(-(LEGLINK1*S2715[2][1]) + WAISTLINK2*S2715[2][3]);
a27[6]=LEGLINK1*a15[1] - WAISTLINK2*a15[3] + a15[5];

a28[1]=c28[1] + a27[1]*S2827[1][1] + a27[3]*S2827[1][3];
a28[2]=c28[2] + a27[1]*S2827[2][1] + a27[3]*S2827[2][3];
a28[3]=state[12].thdd - a27[2];
a28[4]=c28[4] + a27[4]*S2827[1][1] + a27[6]*S2827[1][3];
a28[5]=c28[5] + a27[4]*S2827[2][1] + a27[6]*S2827[2][3];
a28[6]=-a27[5];

a29[1]=c29[1] + a28[1]*S2928[1][1] + a28[3]*S2928[1][3];
a29[2]=c29[2] + a28[1]*S2928[2][1] + a28[3]*S2928[2][3];
a29[3]=state[13].thdd - a28[2];
a29[4]=c29[4] + a28[4]*S2928[1][1] + a28[6]*S2928[1][3];
a29[5]=c29[5] + a28[4]*S2928[2][1] + a28[6]*S2928[2][3];
a29[6]=-a28[5];

a30[1]=c30[1] + a29[1]*S3029[1][1] + a29[2]*S3029[1][2];
a30[2]=c30[2] + a29[1]*S3029[2][1] + a29[2]*S3029[2][2];
a30[3]=state[14].thdd + a29[3];
a30[4]=c30[4] + a29[4]*S3029[1][1] - LEGLINK2*a29[3]*S3029[1][2] + a29[5]*S3029[1][2];
a30[5]=c30[5] + a29[4]*S3029[2][1] - LEGLINK2*a29[3]*S3029[2][2] + a29[5]*S3029[2][2];
a30[6]=LEGLINK2*a29[2] + a29[6];

a31[1]=c31[1] + a30[1]*S3130[1][1] + a30[2]*S3130[1][2];
a31[2]=c31[2] + a30[1]*S3130[2][1] + a30[2]*S3130[2][2];
a31[3]=state[15].thdd + a30[3];
a31[4]=c31[4] + a30[4]*S3130[1][1] - LEGLINK3*a30[3]*S3130[1][2] + a30[5]*S3130[1][2];
a31[5]=c31[5] + a30[4]*S3130[2][1] - LEGLINK3*a30[3]*S3130[2][2] + a30[5]*S3130[2][2];
a31[6]=LEGLINK3*a30[2] + a30[6];

a32[1]=c32[1] + a31[1]*S3231[1][1] + a31[3]*S3231[1][3];
a32[2]=c32[2] + a31[1]*S3231[2][1] + a31[3]*S3231[2][3];
a32[3]=state[16].thdd + a31[2];
a32[4]=c32[4] + a31[4]*S3231[1][1] + a31[6]*S3231[1][3];
a32[5]=c32[5] + a31[4]*S3231[2][1] + a31[6]*S3231[2][3];
a32[6]=a31[5];

a33[1]=a32[1];
a33[2]=a32[2];
a33[3]=a32[3];
a33[4]=FOOTLLENGLONG*a32[2] + FOOTWIDLONG*a32[3] + a32[4];
a33[5]=-(FOOTLLENGLONG*a32[1]) - LEGLINK4*a32[3] + a32[5];
a33[6]=-(FOOTWIDLONG*a32[1]) + LEGLINK4*a32[2] + a32[6];

a34[1]=a32[1];
a34[2]=a32[2];
a34[3]=a32[3];
a34[4]=FOOTLLENGLONG*a32[2] - FOOTWIDSHORT*a32[3] + a32[4];
a34[5]=-(FOOTLLENGLONG*a32[1]) - LEGLINK4*a32[3] + a32[5];
a34[6]=FOOTWIDSHORT*a32[1] + LEGLINK4*a32[2] + a32[6];

a35[1]=a32[1];
a35[2]=a32[2];
a35[3]=a32[3];
a35[4]=-(FOOTLENGSHORT*a32[2]) - FOOTWIDSHORT*a32[3] + a32[4];
a35[5]=FOOTLENGSHORT*a32[1] - LEGLINK4*a32[3] + a32[5];
a35[6]=FOOTWIDSHORT*a32[1] + LEGLINK4*a32[2] + a32[6];

a36[1]=a32[1];
a36[2]=a32[2];
a36[3]=a32[3];
a36[4]=-(FOOTLENGSHORT*a32[2]) + FOOTWIDLONG*a32[3] + a32[4];
a36[5]=FOOTLENGSHORT*a32[1] - LEGLINK4*a32[3] + a32[5];
a36[6]=-(FOOTWIDLONG*a32[1]) + LEGLINK4*a32[2] + a32[6];

a37[1]=a32[1]*S3732[1][1] + a32[2]*S3732[1][2] + a32[3]*S3732[1][3];
a37[2]=a32[1]*S3732[2][1] + a32[2]*S3732[2][2] + a32[3]*S3732[2][3];
a37[3]=a32[1]*S3732[3][1] + a32[2]*S3732[3][2] + a32[3]*S3732[3][3];
a37[4]=a32[4]*S3732[1][1] + a32[5]*S3732[1][2] + a32[3]*(-(eff[2].x[2]*S3732[1][1]) + eff[2].x[1]*S3732[1][2]) + a32[6]*S3732[1][3] + a32[2]*(eff[2].x[3]*S3732[1][1] - eff[2].x[1]*S3732[1][3]) + a32[1]*(-(eff[2].x[3]*S3732[1][2]) + eff[2].x[2]*S3732[1][3]);
a37[5]=a32[4]*S3732[2][1] + a32[5]*S3732[2][2] + a32[3]*(-(eff[2].x[2]*S3732[2][1]) + eff[2].x[1]*S3732[2][2]) + a32[6]*S3732[2][3] + a32[2]*(eff[2].x[3]*S3732[2][1] - eff[2].x[1]*S3732[2][3]) + a32[1]*(-(eff[2].x[3]*S3732[2][2]) + eff[2].x[2]*S3732[2][3]);
a37[6]=a32[4]*S3732[3][1] + a32[5]*S3732[3][2] + a32[3]*(-(eff[2].x[2]*S3732[3][1]) + eff[2].x[1]*S3732[3][2]) + a32[6]*S3732[3][3] + a32[2]*(eff[2].x[3]*S3732[3][1] - eff[2].x[1]*S3732[3][3]) + a32[1]*(-(eff[2].x[3]*S3732[3][2]) + eff[2].x[2]*S3732[3][3]);

/* inverse dynamics torques */
state[22].uff=p2[6] + a2[1]*JA2[6][1] + a2[2]*JA2[6][2] + a2[3]*JA2[6][3] + a2[4]*JA2[6][4] + a2[5]*JA2[6][5];
state[23].uff=p3[6] + a3[1]*JA3[6][1] + a3[2]*JA3[6][2] + a3[3]*JA3[6][3] + a3[4]*JA3[6][4] + a3[5]*JA3[6][5] + a3[6]*JA3[6][6];
state[24].uff=p4[4] + a4[1]*JA4[4][1] + a4[2]*JA4[4][2] + a4[3]*JA4[4][3] + a4[5]*JA4[4][5] + a4[6]*JA4[4][6];
state[7].uff=p5[6] + a5[1]*JA5[6][1] + a5[2]*JA5[6][2] + a5[3]*JA5[6][3] + a5[4]*JA5[6][4] + a5[5]*JA5[6][5];
state[8].uff=p6[6] + a6[1]*JA6[6][1] + a6[2]*JA6[6][2] + a6[3]*JA6[6][3] + a6[4]*JA6[6][4] + a6[5]*JA6[6][5] + a6[6]*JA6[6][6];
state[9].uff=p7[4] + a7[1]*JA7[4][1] + a7[2]*JA7[4][2] + a7[3]*JA7[4][3] + a7[4]*JA7[4][4] + a7[5]*JA7[4][5] + a7[6]*JA7[4][6];
state[10].uff=p8[6] + a8[1]*JA8[6][1] + a8[2]*JA8[6][2] + a8[3]*JA8[6][3] + a8[4]*JA8[6][4] + a8[5]*JA8[6][5];
state[17].uff=p10[6] + a10[1]*JA10[6][1] + a10[2]*JA10[6][2] + a10[3]*JA10[6][3] + a10[4]*JA10[6][4] + a10[5]*JA10[6][5];
state[18].uff=p11[6] + a11[1]*JA11[6][1] + a11[2]*JA11[6][2] + a11[3]*JA11[6][3] + a11[4]*JA11[6][4] + a11[5]*JA11[6][5] + a11[6]*JA11[6][6];
state[19].uff=p12[4] + a12[1]*JA12[4][1] + a12[2]*JA12[4][2] + a12[3]*JA12[4][3] + a12[4]*JA12[4][4] + a12[5]*JA12[4][5] + a12[6]*JA12[4][6];
state[20].uff=p13[6] + a13[1]*JA13[6][1] + a13[2]*JA13[6][2] + a13[3]*JA13[6][3] + a13[4]*JA13[6][4] + a13[5]*JA13[6][5];
state[21].uff=p15[6] + a15[1]*JA15[6][1] + a15[2]*JA15[6][2] + a15[3]*JA15[6][3] + a15[4]*JA15[6][4] + a15[5]*JA15[6][5];
state[1].uff=p16[6] + a16[1]*JA16[6][1] + a16[2]*JA16[6][2] + a16[3]*JA16[6][3] + a16[4]*JA16[6][4] + a16[5]*JA16[6][5] + a16[6]*JA16[6][6];
state[2].uff=p17[6] + a17[1]*JA17[6][1] + a17[2]*JA17[6][2] + a17[3]*JA17[6][3] + a17[4]*JA17[6][4] + a17[5]*JA17[6][5] + a17[6]*JA17[6][6];
state[3].uff=p18[6] + a18[1]*JA18[6][1] + a18[2]*JA18[6][2] + a18[3]*JA18[6][3] + a18[4]*JA18[6][4] + a18[5]*JA18[6][5] + a18[6]*JA18[6][6];
state[4].uff=p19[6] + a19[1]*JA19[6][1] + a19[2]*JA19[6][2] + a19[3]*JA19[6][3] + a19[4]*JA19[6][4] + a19[5]*JA19[6][5] + a19[6]*JA19[6][6];
state[5].uff=p20[6] + a20[1]*JA20[6][1] + a20[2]*JA20[6][2] + a20[3]*JA20[6][3] + a20[4]*JA20[6][4] + a20[5]*JA20[6][5] + a20[6]*JA20[6][6];
state[6].uff=p21[6] + a21[1]*JA21[6][1] + a21[2]*JA21[6][2] + a21[3]*JA21[6][3] + a21[4]*JA21[6][4] + a21[5]*JA21[6][5] + a21[6]*JA21[6][6];
state[11].uff=p27[6] + a27[1]*JA27[6][1] + a27[2]*JA27[6][2] + a27[3]*JA27[6][3] + a27[4]*JA27[6][4] + a27[5]*JA27[6][5] + a27[6]*JA27[6][6];
state[12].uff=p28[6] + a28[1]*JA28[6][1] + a28[2]*JA28[6][2] + a28[3]*JA28[6][3] + a28[4]*JA28[6][4] + a28[5]*JA28[6][5] + a28[6]*JA28[6][6];
state[13].uff=p29[6] + a29[1]*JA29[6][1] + a29[2]*JA29[6][2] + a29[3]*JA29[6][3] + a29[4]*JA29[6][4] + a29[5]*JA29[6][5] + a29[6]*JA29[6][6];
state[14].uff=p30[6] + a30[1]*JA30[6][1] + a30[2]*JA30[6][2] + a30[3]*JA30[6][3] + a30[4]*JA30[6][4] + a30[5]*JA30[6][5] + a30[6]*JA30[6][6];
state[15].uff=p31[6] + a31[1]*JA31[6][1] + a31[2]*JA31[6][2] + a31[3]*JA31[6][3] + a31[4]*JA31[6][4] + a31[5]*JA31[6][5] + a31[6]*JA31[6][6];
state[16].uff=p32[6] + a32[1]*JA32[6][1] + a32[2]*JA32[6][2] + a32[3]*JA32[6][3] + a32[4]*JA32[6][4] + a32[5]*JA32[6][5] + a32[6]*JA32[6][6];

