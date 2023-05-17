#ifndef PID_H
#define PID_H

class PID {
    public:
        PID(float Kp, float Kd, float Ki, float target);        
        float getControl(float value, float dt_seconds);
        void setSettings(float Kp, float Kd, float Ki);
        void setTarget(float target);
    private:
        float _Kp;
        float _Kd;
        float _Ki;
        float _lastError;
        float _integralError;
        float _target;
        float _last_value;
        bool _has_last_value = false;
};

PID::PID(float Kp, float Kd, float Ki, float target) {
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    _target = target;
}

float PID::getControl(float value, float dt_seconds) {
    float lastValue = _has_last_value ? _last_value : value;
    float error = _target - value;
    float de = -(value - lastValue) / dt_seconds;
    _integralError += _Ki * error * dt_seconds;
    _lastError = error;
    _last_value = value;
    _has_last_value = true;
    return (_Kp * error + _Kd * de + _integralError);
}

void PID::setSettings(float Kp, float Kd, float Ki) {
  _Kp = Kp;
  _Kd = Kd;
  _Ki = Ki;
}

void PID::setTarget(float target) {
    _target = target;
    _integralError = .0;
}

#endif