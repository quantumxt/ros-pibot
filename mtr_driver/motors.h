class MotorControl {
  public:
    //Var
    int maxSpd = 255;
    float moveFactor = 1.0;
    float turnFactor = 3.0;

    MotorControl(int enA, int enB, int in1, int in2, int in3, int in4);
    void initMotors();
    void rotateBot(bool dir, float spd);
    void moveBot(float spd, float bias);
    void stopMotors();
  private:
    //The pins
    int enA;
    int enB;
    int in1;
    int in2;
    int in3;
    int in4;


};
