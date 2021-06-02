
#ifndef FIRFILT_H
#define FIRFILT_H

/* .......................................................................... */

typedef struct
{
  int  numTaps;
  int *pTapsBase;
  int *pTapsEnd;
  int  tapsPage;
  int *pDelayBase;
  int *pDelayEnd;
  int *pDelayPtr;
} FIRStruct;


extern void FIR( int, int *, int *, FIRStruct * );
extern void FIRDelayInit( FIRStruct * );

/* .......................................................................... */

#endif /* FIRFILT_H*/
