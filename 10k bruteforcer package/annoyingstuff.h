struct floor {
    float one;
    double *two;
};

struct importantshit {
    float speed;
    signed short slidingyaw;
    float magused1, magused2;
    signed short angleused1, angleused2;
    
    int zzzzz;
};

struct MarioState {
    float slideVelX, slideVelZ;
    float forwardVel;
    signed short slideYaw;
    signed short faceAngle;

    float floornormalz, floornormalx;
    int floorclass;

    float intendedMag;
    signed short intendedYaw;

    float vel[3];
};



void solution_to_file(struct importantshit *, FILE *);

signed short atan2_lookup(float, float);
signed short atan2s(float, float);
float cosine(signed short);
float sine(signed short);

int lower_index(signed short);
int higher_index(signed short);
double min_3(double, double, double);
double max_3(double, double, double);
void cellstart(int [][64]);
void cells(double *[][64][21], int [][64], double *, int);
void cells2(int [][64][21], int [][64], double *, int);

struct floor find_any_floor(signed long, signed long, signed long);

void update_sliding_angle(struct MarioState *, float, float);
signed short update_sliding(struct MarioState *, float);
int should_slide(float, int);
int floor_is_slope(float, int);

void gpu_setup(cl_command_queue *, cl_kernel *, cl_mem[5]);
void slidekick_second_freefall(float, signed short, float, float, float, struct importantshit *, FILE *);
void slidekick_tenk_stuff(signed short, float, float, float, float, struct importantshit *, FILE *);
void gpu_slidekick_crouch_slide(float, signed short, struct importantshit *, FILE *, cl_command_queue *, cl_kernel *, cl_mem[5]);
void gpu_slidekick_start(signed short, FILE *);

