#ifndef FILTERED_DERIVATIVE_WCB_H
#define FILTERED_DERIVATIVE_WCB_H

#include <iostream>
#include "LowPassFilter.h"
#include "circular_buffer.h"
#include "sys/time.h"

#define FD_WCB_CB_MIN_CAP 20

namespace CVG_BlockDiagram {

typedef struct {
  timeval tv;
  double  d;
} StampedDouble;
typedef circular_buffer<StampedDouble> cbuf_type;

class FilteredDerivativeWCB {
private:
    cbuf_type     circular_buffer;
    LowPassFilter lowpassfilter_pre, lowpassfilter_post;
    double        Tderiv;
    double        last_output;

public:
    FilteredDerivativeWCB();
    ~FilteredDerivativeWCB();

    void reset();

    void setTimeParameters( double preTr, double postTr, double Tderiv_, double Tmemory, double SensorFreq);

    void setInput(double x_k, time_t tv_sec, suseconds_t tv_usec);
    void getOutput(double &x_tf, double &dx_tf);
};

}

#endif // FILTERED_DERIVATIVE_WCB_H
