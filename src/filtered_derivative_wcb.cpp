#include "filtered_derivative_wcb.h"

namespace CVG_BlockDiagram {

FilteredDerivativeWCB::FilteredDerivativeWCB() {
    lowpassfilter_pre.setResponseTime( 1.0);
    lowpassfilter_pre.reset();
    lowpassfilter_post.setResponseTime( 1.0);
    lowpassfilter_post.reset();

    last_output = 0.0;
}

FilteredDerivativeWCB::~FilteredDerivativeWCB()
{
}

void FilteredDerivativeWCB::reset() {
    lowpassfilter_pre.reset();
    lowpassfilter_post.reset();
}

void FilteredDerivativeWCB::setTimeParameters(double preTr, double postTr, double Tderiv, double Tmemory, double SensorFreq)
{
    lowpassfilter_pre.setResponseTime(preTr);
    lowpassfilter_post.setResponseTime(postTr);
    // Note that Tmemory has to be longer than Tderiv
    this->Tderiv = Tderiv;

    long cbuf_capacity = (Tmemory*SensorFreq > FD_WCB_CB_MIN_CAP) ? Tmemory*SensorFreq : FD_WCB_CB_MIN_CAP;
    circular_buffer.reserve( cbuf_capacity );
}

void FilteredDerivativeWCB::setInput(double x_t, time_t tv_sec, suseconds_t tv_usec) {
    lowpassfilter_pre.setInput(x_t);
    double x_tf = lowpassfilter_pre.getOutput();

    StampedDouble x_tf_tv;
    x_tf_tv.d          = x_tf;
    x_tf_tv.tv.tv_sec  = tv_sec;
    x_tf_tv.tv.tv_usec = tv_usec;

    circular_buffer.push_back(x_tf_tv);
}

void FilteredDerivativeWCB::getOutput(double &x_tf, double &dx_tf) {
//    timeval tv_t; gettimeofday(&tv_t, NULL);
//    x_tf = lowpassfilter.getOutput();
    StampedDouble x_tf_aux = circular_buffer.back();
    timeval tv_t;
    tv_t.tv_sec  = x_tf_aux.tv.tv_sec;
    tv_t.tv_usec = x_tf_aux.tv.tv_usec;
    x_tf = x_tf_aux.d;

    double x_tmTderivf     = x_tf;
    double elapsed_seconds = 0.0;

//    bool found_x_tmTderiv_f = false;
    for (cbuf_type::reverse_iterator ri = circular_buffer.rbegin(); ri != circular_buffer.rend(); ri++) {
        elapsed_seconds = (tv_t.tv_sec - (*ri).tv.tv_sec) + (tv_t.tv_usec - (*ri).tv.tv_usec) / 1e6;
        x_tmTderivf = (*ri).d;

        if (elapsed_seconds > Tderiv) {
//            found_x_tmTderiv_f = true;
            break;
        }
    }

if (elapsed_seconds != 0.0) {
        last_output = (x_tf - x_tmTderivf)/elapsed_seconds;
    }
    dx_tf = last_output;

    lowpassfilter_post.setInput(dx_tf);
    dx_tf = lowpassfilter_post.getOutput();
/*
    std::cout << "x_tf:" << x_tf <<
                 "\t x_tmT:" << x_tmTderivf <<
                 "\t dx_tf:" << dx_tf <<
                 "\t DT:" << elapsed_seconds <<
                 "\t sec:" << tv_t.tv_sec   <<
                 "\t usec:" << tv_t.tv_usec << std::endl;
*/
    return;
}

}
