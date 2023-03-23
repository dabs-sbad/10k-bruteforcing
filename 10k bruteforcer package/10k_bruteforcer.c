#include<stdio.h>
#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif
#include<stdlib.h>
#include<math.h>
#include<string.h>
#include"annoyingstuff.h"
#include"datathingys.c"
#include<stdbool.h>
#include"elevator1.c"
#include"elevator2.c"

#define current_angle_for_testing -26128

#define H_speed 20
#define facing_angle -26262

#define mxstart -5229.51904296875f
#define mystart -158.473190307617f
#define mzstart -5904.19140625f




double *Alltrigscells[64][64][21];
int Alltrigscells2[64][64][21];
int Alltrigscellsnumber[64][64];



int main(void) {

    int Alltrigsnumber = ((sizeof(Alltrigs)/sizeof(Alltrigs[0])) - 1) / 14;

    FILE *secondfall;
    secondfall = fopen("10k solutions.txt", "w");
    
    cellstart(Alltrigscellsnumber);
    cells(Alltrigscells, Alltrigscellsnumber, Alltrigs, Alltrigsnumber);
    cellstart(Alltrigscellsnumber);
    cells2(Alltrigscells2, Alltrigscellsnumber, Alltrigs, Alltrigsnumber);
  


    fprintf(secondfall, "angle %d start\n", current_angle_for_testing);
    gpu_slidekick_start(current_angle_for_testing, secondfall);
    fprintf(secondfall, "angle %d end\n", current_angle_for_testing);



    //printf("%d\n", atan2s(cosine(-8192), sine(-8192)));
    //printf("%d\n", atan2s(524268.219, -524268.219));

    return 0;
}



void solution_to_file(struct importantshit *important, FILE *file) {

    fprintf(file, "%0.2ff, ", important->speed);
    fprintf(file, "%0.9ff, ", important->magused1);
    fprintf(file, "%d, ", important->angleused1);
    fprintf(file, "%0.9ff, ", important->magused2);
    fprintf(file, "%d, \n", important->angleused2);




    //fprintf(file, "%d, ", important->angle1);
    //fprintf(file, "%d, ", important->angle2);
    //fprintf(file, "%d, ", a.x);
    //fprintf(file, "%d, ", a.y);
    //fprintf(file, "%d", important->frictionframes);
}



signed short atan2_lookup(float y, float x) {
    unsigned short ret;

    if (x == 0) {
        ret = gArctanTable[0];
    } else {
        ret = gArctanTable[(signed short)(y / x * 1024.0f + 0.5f)];
    }
    return ret;
}

signed short atan2s(float y, float x) {
    unsigned short ret;

    if (x >= 0) {
        if (y >= 0) {
            if (y >= x) {
                ret = atan2_lookup(x, y);
            } else {
                ret = 0x4000 - atan2_lookup(y, x);
            }
        } else {
            y = -y;
            if (y < x) {
                ret = 0x4000 + atan2_lookup(y, x);
            } else {
                ret = 0x8000 - atan2_lookup(x, y);
            }
        }
    } else {
        x = -x;
        if (y < 0) {
            y = -y;
            if (y >= x) {
                ret = 0x8000 + atan2_lookup(x, y);
            } else {
                ret = 0xC000 - atan2_lookup(y, x);
            }
        } else {
            if (y < x) {
                ret = 0xC000 + atan2_lookup(y, x);
            } else {
                ret = -1 * atan2_lookup(x, y);
            }
        }
    }
    return ret;
}

float cosine(signed short x) {
    return gCosineTable[(unsigned short) (x) >> 4];
}

float sine(signed short x) {
    return cosine(x - 16384);
}



int lower_index(signed short coord) {
    int index;

    coord += 8192;
    if (coord < 0) {
        coord = 0;
    }

    index = coord / 256;

    if (index < 0) {
        index = 0;
    }

    return index;
}

int higher_index(signed short coord) {
    int index;

    coord += 8192;
    if (coord < 0) {
        coord = 0;
    }

    index = coord / 256;

    if (index > 63) {
        index = 63;
    }

    return index;
}

double min_3(double a0, double a1, double a2) {
    if (a1 < a0) {
        a0 = a1;
    }

    if (a2 < a0) {
        a0 = a2;
    }

    return a0;
}

double max_3(double a0, double a1, double a2) {
    if (a1 > a0) {
        a0 = a1;
    }

    if (a2 > a0) {
        a0 = a2;
    }

    return a0;
}

void cellstart(int cellsnumber[][64]) {
    int i, j;
    for (i = 0; i < 64; i++) {
        for (j = 0; j < 64; j++) {
            cellsnumber[i][j] = 0;
        }
    }
}

void cells(double *cells[][64][21], int cellsnumber[][64], double *triangles, int number) {  
    int i, index;
    int maxcellx,  maxcelly, mincellx, mincelly, cellx, celly;
    signed short maxx, maxz, minx, minz;
    for (i = 0; i < number; i++) {
        maxx = (signed short) max_3(triangles[(14 * i) + 1], triangles[(14 * i) + 4], triangles[(14 * i) + 7]);
        minx = (signed short) min_3(triangles[(14 * i) + 1], triangles[(14 * i) + 4], triangles[(14 * i) + 7]);
        maxz = (signed short) max_3(triangles[(14 * i) + 3], triangles[(14 * i) + 6], triangles[(14 * i) + 9]);
        minz = (signed short) min_3(triangles[(14 * i) + 3], triangles[(14 * i) + 6], triangles[(14 * i) + 9]);
        maxcellx = higher_index(maxx);
        mincellx = lower_index(minx);
        maxcelly = higher_index(maxz);
        mincelly = lower_index(minz);

        for (cellx = mincellx; cellx <= maxcellx; cellx++) {
           for (celly = mincelly; celly <= maxcelly; celly++) {
               index = cellsnumber[cellx][celly];
               if (index >= 20) {
                   printf("mission failed! we'll get them next time\n");
                   printf("pointer table overflowed, not pog");
                   exit(1);
               }
               cells[cellx][celly][index] = &triangles[(14 * i)];
               cellsnumber[cellx][celly]++;
           }
        }
    }
}

void cells2(int cells[][64][21], int cellsnumber[][64], double *triangles, int number) {  
    int i, index;
    int maxcellx,  maxcelly, mincellx, mincelly, cellx, celly;
    signed short maxx, maxz, minx, minz;
    for (i = 0; i < number; i++) {
        maxx = (signed short) max_3(triangles[(14 * i) + 1], triangles[(14 * i) + 4], triangles[(14 * i) + 7]);
        minx = (signed short) min_3(triangles[(14 * i) + 1], triangles[(14 * i) + 4], triangles[(14 * i) + 7]);
        maxz = (signed short) max_3(triangles[(14 * i) + 3], triangles[(14 * i) + 6], triangles[(14 * i) + 9]);
        minz = (signed short) min_3(triangles[(14 * i) + 3], triangles[(14 * i) + 6], triangles[(14 * i) + 9]);
        maxcellx = higher_index(maxx);
        mincellx = lower_index(minx);
        maxcelly = higher_index(maxz);
        mincelly = lower_index(minz);

        for (cellx = mincellx; cellx <= maxcellx; cellx++) {
           for (celly = mincelly; celly <= maxcelly; celly++) {
               index = cellsnumber[cellx][celly];
               if (index >= 20) {
                   printf("mission failed! we'll get them next time\n");
                   printf("pointer table overflowed, not pog");
                   exit(1);
               }
               cells[cellx][celly][index] = 14 * i;
               cellsnumber[cellx][celly]++;
           }
        }
    }
}



struct floor find_any_floor(signed long x, signed long y, signed long z) {
    int cellx, celly;

    x = (signed short) x;
    y = (signed short) y;
    z = (signed short) z;

    struct floor result;
    result.one = -11000;
    result.two = NULL;

    if (x < -8192 || x > 8192) {
        return result;
    }
    if (z < -8192 || z > 8192) {
        return result;
    }

    cellx = ((x + 8192) / 256) & 63;
    celly = ((z + 8192) / 256) & 63;

    int i;
    signed long x1, z1, x2, z2, x3, z3;
    float nx, ny, nz;
    float oo;
    float height;

    for (i = 0; i < Alltrigscellsnumber[cellx][celly]; i++) {
        x1 = (signed long) *(Alltrigscells[cellx][celly][i] + 1);
        z1 = (signed long) *(Alltrigscells[cellx][celly][i] + 3);
        x2 = (signed long) *(Alltrigscells[cellx][celly][i] + 4);
        z2 = (signed long) *(Alltrigscells[cellx][celly][i] + 6);
        if ((z1 - z) * (x2 - x1) - (x1 - x) * (z2 - z1) < 0) {
            continue;
        }

        x3 = *(Alltrigscells[cellx][celly][i] + 7);
        z3 = *(Alltrigscells[cellx][celly][i] + 9);

        if ((z2 - z) * (x3 - x2) - (x2 - x) * (z3 - z2) < 0) {
            continue;
        }
        if ((z3 - z) * (x1 - x3) - (x3 - x) * (z1 - z3) < 0) {
            continue;
        }
        nx = *(Alltrigscells[cellx][celly][i] + 10);
        ny = *(Alltrigscells[cellx][celly][i] + 11);
        nz = *(Alltrigscells[cellx][celly][i] + 12);
        oo = *(Alltrigscells[cellx][celly][i] + 13);

        height = -(x * nx + nz * z + oo) / ny;

        if (y - (height + -78.0f) < 0.0f) {
            continue;
        }
        result.one = height;
        result.two = Alltrigscells[cellx][celly][i];
        return result;
    }
    return result;
}



void update_sliding_angle(struct MarioState *m, float accel, float lossFactor) {
    signed int newFacingDYaw;
    signed short facingDYaw;

    signed short slopeAngle = atan2s(m->floornormalz, m->floornormalx);
    float steepness = sqrtf(m->floornormalx * m->floornormalx + m->floornormalz * m->floornormalz);
    //UNUSED f32 normalY = floor->normal.y;

    m->slideVelX += accel * steepness * sine(slopeAngle);
    m->slideVelZ += accel * steepness * cosine(slopeAngle);

    m->slideVelX *= lossFactor;
    m->slideVelZ *= lossFactor;

    m->slideYaw = atan2s(m->slideVelZ, m->slideVelX);

    facingDYaw = m->faceAngle - m->slideYaw;
    newFacingDYaw = facingDYaw;

    //! -0x4000 not handled - can slide down a slope while facing perpendicular to it
    if (newFacingDYaw > 0 && newFacingDYaw <= 0x4000) {
        if ((newFacingDYaw -= 0x200) < 0) {
            newFacingDYaw = 0;
        }
    } else if (newFacingDYaw > -0x4000 && newFacingDYaw < 0) {
        if ((newFacingDYaw += 0x200) > 0) {
            newFacingDYaw = 0;
        }
    } else if (newFacingDYaw > 0x4000 && newFacingDYaw < 0x8000) {
        if ((newFacingDYaw += 0x200) > 0x8000) {
            newFacingDYaw = 0x8000;
        }
    } else if (newFacingDYaw > -0x8000 && newFacingDYaw < -0x4000) {
        if ((newFacingDYaw -= 0x200) < -0x8000) {
            newFacingDYaw = -0x8000;
        }
    }

    m->faceAngle = m->slideYaw + newFacingDYaw;

    m->vel[0] = m->slideVelX;
    m->vel[1] = 0.0f;
    m->vel[2] = m->slideVelZ;

    //mario_update_moving_sand(m);
    //mario_update_windy_ground(m);

    //! Speed is capped a frame late (butt slide HSG)
    m->forwardVel = sqrtf(m->slideVelX * m->slideVelX + m->slideVelZ * m->slideVelZ);
    if (m->forwardVel > 100.0f) {
        m->slideVelX = m->slideVelX * 100.0f / m->forwardVel;
        m->slideVelZ = m->slideVelZ * 100.0f / m->forwardVel;
    }

    if (newFacingDYaw < -0x4000 || newFacingDYaw > 0x4000) {
        m->forwardVel *= -1.0f;
    }
}

signed short update_sliding(struct MarioState *m, float stopSpeed) {
    float lossFactor;
    float accel;
    float oldSpeed;
    float newSpeed;

    signed int stopped = 0;

    signed short intendedDYaw = m->intendedYaw - m->slideYaw;
    float forward = cosine(intendedDYaw);
    float sideward = sine(intendedDYaw);

    //! 10k glitch
    if (forward < 0.0f && m->forwardVel >= 0.0f) {
        forward *= 0.5f + 0.5f * m->forwardVel / 100.0f;
    }

    switch(m->floorclass) {
        case 4: //very slippery
            accel = 10.0f;
            lossFactor = m->intendedMag / 32.0f * forward * 0.02f + 0.98f;
            break;

        case 3: //slippery
            accel = 8.0f;
            lossFactor = m->intendedMag / 32.0f * forward * 0.02f + 0.96f;
            break;

        default:
            accel = 7.0f;
            lossFactor = m->intendedMag / 32.0f * forward * 0.02f + 0.92f;
            break;

        case 1: //non sliperry
            accel = 5.0f;
            lossFactor = m->intendedMag / 32.0f * forward * 0.02f + 0.92f;
            break;
    }

    oldSpeed = sqrtf(m->slideVelX * m->slideVelX + m->slideVelZ * m->slideVelZ);

    //! This is attempting to use trig derivatives to rotate Mario's speed.
    // It is slightly off/asymmetric since it uses the new X speed, but the old
    // Z speed.
    m->slideVelX += m->slideVelZ * (m->intendedMag / 32.0f) * sideward * 0.05f;
    m->slideVelZ -= m->slideVelX * (m->intendedMag / 32.0f) * sideward * 0.05f;

    newSpeed = sqrtf(m->slideVelX * m->slideVelX + m->slideVelZ * m->slideVelZ);

    if (oldSpeed > 0.0f && newSpeed > 0.0f) {
        m->slideVelX = m->slideVelX * oldSpeed / newSpeed;
        m->slideVelZ = m->slideVelZ * oldSpeed / newSpeed;
    }

    update_sliding_angle(m, accel, lossFactor);

    //if (!mario_floor_is_slope(m) && m->forwardVel * m->forwardVel < stopSpeed * stopSpeed) {
    //    mario_set_forward_vel(m, 0.0f);
    //    stopped = 1;
    //}

    return stopped;
}

int should_slide(float ynorm, int class) {

    float normY;

    switch(class) {
                
        case 4:
            normY = 0.9848077f;
            break;

        case 3:
            normY = 0.9396926f;
            break;

        default:
            normY = 0.7880108f;
            break;

        case 1:
            normY = 0.0f;
            break;
    }

    return ynorm <= normY;
}

int floor_is_slope(float ynorm, int class) {

    float normY;

    switch(class) {
                
        case 4:
            normY = 0.9961947f;
            break;

        case 3:
            normY = 0.9848077f;
            break;

        default:
            normY = 0.9659258f;
            break;

        case 1:
            normY = 0.9396926f;
            break;
    }

    return ynorm <= normY;
}



void gpu_setup(cl_command_queue *queue_out, cl_kernel *kernel_out, cl_mem stuff_out[5]) {

    FILE *fp;
    char *source_str;
    size_t source_size;
 
    fp = fopen("10k_kernel.cl", "r");
    if (!fp) {
        fprintf(stderr, "Failed to load kernel.\n");
        exit(1);
    }
    source_str = (char*)malloc(0x100000);
    source_size = fread(source_str, 1, 0x100000, fp);
    fclose(fp);
 
    // Get platform and device information
    cl_platform_id platform_id = NULL;
    cl_device_id device_id = NULL;   
    cl_uint ret_num_devices;
    cl_uint ret_num_platforms;
    cl_int ret = clGetPlatformIDs(1, &platform_id, &ret_num_platforms);
    ret = clGetDeviceIDs( platform_id, CL_DEVICE_TYPE_GPU, 1, &device_id, &ret_num_devices);

    // Create an OpenCL context
    cl_context context = clCreateContext( NULL, 1, &device_id, NULL, NULL, &ret);

    // Create a command queue
    cl_command_queue command_queue = clCreateCommandQueue(context, device_id, 0, &ret);
 
    // Create memory buffers on the device for each vector 
    cl_mem allFloors_mem_obj = clCreateBuffer(context, CL_MEM_READ_ONLY, 86016 * sizeof(int), NULL, &ret);
    cl_mem allFloorsNumbers_mem_obj = clCreateBuffer(context, CL_MEM_READ_ONLY, 4096 * sizeof(int), NULL, &ret);
    cl_mem Marioints_mem_obj = clCreateBuffer(context, CL_MEM_READ_ONLY, 2 * sizeof(int), NULL, &ret);
    cl_mem Mariofloats_mem_obj = clCreateBuffer(context, CL_MEM_READ_ONLY, 4 * sizeof(float), NULL, &ret);
    cl_mem results_mem_obj = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(int), NULL, &ret);
 
    // Copy the lists A and B to their respective memory buffers
    ret = clEnqueueWriteBuffer(command_queue, allFloors_mem_obj, CL_TRUE, 0, 86016 * sizeof(int), Alltrigscells2, 0, NULL, NULL);
    ret = clEnqueueWriteBuffer(command_queue, allFloorsNumbers_mem_obj, CL_TRUE, 0, 4096 * sizeof(int), Alltrigscellsnumber, 0, NULL, NULL);
    ret = clEnqueueWriteBuffer(command_queue, results_mem_obj, CL_TRUE, 0, sizeof(int), 0, 0, NULL, NULL);
    
    // Create a program from the kernel source
    cl_program program = clCreateProgramWithSource(context, 1, (const char **)&source_str, (const size_t *)&source_size, &ret);
 
    // Build the program
    ret = clBuildProgram(program, 1, &device_id, NULL, NULL, NULL);
 
    // Create the OpenCL kernel
    cl_kernel kernel = clCreateKernel(program, "tenk_kernel", &ret);

    *queue_out = command_queue;
    *kernel_out = kernel;
    stuff_out[0] = allFloors_mem_obj;
    stuff_out[1] = allFloorsNumbers_mem_obj;
    stuff_out[2] = Marioints_mem_obj;
    stuff_out[3] = Mariofloats_mem_obj;
    stuff_out[4] = results_mem_obj;

    clFinish(command_queue);

    return;
}

void slidekick_second_freefall(float speed, signed short angle, float x, float y, float z, struct importantshit *important, FILE *secondfall) {

    float x2, y2, z2;
    struct floor floor1;
    int i;

    x2 = x;
    y2 = y;
    z2 = z;

    for (i = 0; i < 1; i++) {

        x2 = x2 + (speed * sine(angle) * 0.25f);
        z2 = z2 + (speed * cosine(angle) * 0.25f);

        floor1 = find_any_floor(x2, y2, z2);

        if (floor1.two == NULL) {
            break;
        }

        if (floor1.one < y2) {
            continue;
        }

        if (*(floor1.two + 11) < 0.2923717f) {
            break;
        }

        if (should_slide(*(floor1.two + 11), *(floor1.two + 14))) {
            break;
        }



        solution_to_file(important, secondfall);

        break;
    }
}

void slidekick_tenk_stuff(signed short angle, float speed, float x, float y, float z, struct importantshit *important, FILE *secondfall) {
    struct MarioState mario;
    struct MarioState *m = &mario;
    int j, c, a, ma;
    float newspeed;

    float x2, y2, z2;
    float slidex, slidez, defacto, camx, camz;
    struct floor floor1, first;

    signed short camerayaw, slideangle;

    floor1 = find_any_floor(x, y, z);

    m->slideVelX = speed * sine(angle);
    m->slideVelZ = speed * cosine(angle);
    m->forwardVel = speed;
    m->slideYaw = important->slidingyaw;
    m->faceAngle = angle;

    m->floornormalx = *(floor1.two + 10);
    m->floornormalz = *(floor1.two + 12);
    m->floorclass = *(floor1.two + 14);

    for (ma = 0; ma < 1168; ma++) {
        for (a = -2048; a < 2048; a++) {
        
        m->slideVelX = speed * sine(angle);
        m->slideVelZ = speed * cosine(angle);
        m->forwardVel = speed;
        m->slideYaw = important->slidingyaw;
        m->faceAngle = angle;

        m->floornormalx = *(floor1.two + 10);
        m->floornormalz = *(floor1.two + 12);
        m->floorclass = *(floor1.two + 14);

        m->vel[0] = 0;
        m->vel[1] = 0;
        m->vel[2] = 0;

        m->intendedMag = AllMags[ma];
        m->intendedYaw = a * 16;

        update_sliding(m, 0);

        if (m->forwardVel > 0.0f) {
            continue;
        }

        if (m->forwardVel < -1.0f * speed) {
            continue;
        }

        if (m->forwardVel > -65536) {
            continue;
        }

        x2 = x;
        y2 = y;
        z2 = z;
        defacto = *(floor1.two + 11);

        slidex = m->vel[0];
        slidez = m->vel[2];


        for (j = 0; j < 1; j++) {

            //spent so long working with only qspeed I forgor how to regular speed :(
            //x2 = x2 + (slidex * defacto);
            //z2 = z2 + (slidez * defacto);
            
            x2 = x2 + (slidex * 0.25f * defacto);
            z2 = z2 + (slidez * 0.25f * defacto);
            first = find_any_floor(x2, y2, z2);

            if (first.two == NULL) {
                break;
            }
            
            if ((y2 - first.one) < 100) {
                y2 = first.one;
                defacto = *(first.two + 11);
                continue;
            }

            important->magused2 = AllMags[ma];
            important->angleused2 = a * 16;

            slidekick_second_freefall(m->forwardVel, m->faceAngle, x2, y2, z2, important, secondfall);
            break;
        }
        }
    }
}

void gpu_slidekick_crouch_slide(float speed, signed short angle, struct importantshit *important, FILE *secondfall, cl_command_queue *queue, cl_kernel *kernel, cl_mem stuff[5]) {
    
    if (speed > 98895688.0f) {
        exit(0);
    }
    
    
    struct MarioState mario;
    struct MarioState *m = &mario;
    int j, c, ma, a, i, k;
    float newspeed;

    float x2, y2, z2;
    float slidex, slidez, defacto;
    struct floor first, floor1;

    floor1 = find_any_floor(mxstart, mystart, mzstart);

    for (i = 0; i < 4096; i++) {
        for (k = 0; k < 1168; k++) {

        m->slideVelX = speed * sine(angle);
        m->slideVelZ = speed * cosine(angle);
        m->forwardVel = H_speed;
        m->slideYaw = 0;
        m->faceAngle = facing_angle;

        m->floornormalx = *(floor1.two + 10);
        m->floornormalz = *(floor1.two + 12);
        m->floorclass = *(floor1.two + 14);

        m->vel[0] = 0;
        m->vel[1] = 0;
        m->vel[2] = 0;

        m->intendedMag = AllMags[k];
        m->intendedYaw = i * 16;
        
        if (m->forwardVel < 8.0f) {
            if (m->forwardVel < m->intendedMag) {
                m->forwardVel = m->intendedMag;
            }

            if (m->forwardVel > 8.0f) {
                m->forwardVel = 8.0f;
            }
        }

        update_sliding(m, 0);

        if (m->forwardVel < 0.0f) {
            continue;
        }

        x2 = mxstart;
        y2 = mystart;
        z2 = mzstart;
        defacto = *(floor1.two + 11);

        slidex = m->vel[0];
        slidez = m->vel[2];

        x2 = x2 + (slidex * 0.25f * defacto);
        z2 = z2 + (slidez * 0.25f * defacto);

        first = find_any_floor(x2, y2, z2);

        if (first.two != NULL) {
            continue;
        }

        x2 = mxstart;
        y2 = mystart;
        z2 = mzstart;

        m->forwardVel -= 0.35f;
        m->forwardVel -= 1.0f;
        


        for (j = 0; j < 4; j++) {

            //spent so long working with only qspeed I forgor how to regular speed :(
            //x2 = x2 + (slidex * defacto);
            //z2 = z2 + (slidez * defacto);
            
            //dumbass forgets he's no longer sliding
            //x2 = x2 + slidex * 0.25f;
            //z2 = z2 + slidez * 0.25f;

            x2 = x2 + (m->forwardVel * sine(m->faceAngle) * 0.25f);
            y2 = y2 + 3;
            z2 = z2 + (m->forwardVel * cosine(m->faceAngle) * 0.25f);

            first = find_any_floor(x2, y2, z2);

            if (first.two == NULL) {
                break;
            }
            
            if (y2 > first.one) {
                continue;
            }

            y2 = first.one;

            important->speed = speed;
            important->magused1 = m->intendedMag;
            important->angleused1 = m->intendedYaw;
            important->slidingyaw = m->slideYaw;

            //Setup gpu values
            int mInts[2];
            float mFloats[4];
            int ret;

            int result = 0;

            mInts[0] = m->faceAngle;
            mInts[1] = m->slideYaw;

            mFloats[0] = x2;
            mFloats[1] = y2;
            mFloats[2] = z2;
            mFloats[3] = m->forwardVel;

            //Update the buffers
            ret = clEnqueueWriteBuffer(*queue, stuff[2], CL_TRUE, 0, 2 * sizeof(int), mInts, 0, NULL, NULL);
            ret = clEnqueueWriteBuffer(*queue, stuff[3], CL_TRUE, 0, 4 * sizeof(float), mFloats, 0, NULL, NULL);
            ret = clEnqueueWriteBuffer(*queue, stuff[4], CL_TRUE, 0, sizeof(int), &result, 0, NULL, NULL);

            // Set the arguments of the kernel
            ret = clSetKernelArg(*kernel, 0, sizeof(cl_mem), (void *)&stuff[0]);
            ret = clSetKernelArg(*kernel, 1, sizeof(cl_mem), (void *)&stuff[1]);
            ret = clSetKernelArg(*kernel, 2, sizeof(cl_mem), (void *)&stuff[2]);
            ret = clSetKernelArg(*kernel, 3, sizeof(cl_mem), (void *)&stuff[3]);
            ret = clSetKernelArg(*kernel, 4, sizeof(cl_mem), (void *)&stuff[4]);

            // Execute the OpenCL kernel on the list
            size_t global_item_size[2] = {4096, 1168}; // Process the entire lists
            size_t local_item_size[2] = {16, 16}; // Divide work items into groups of 64
            ret = clEnqueueNDRangeKernel(*queue, *kernel, 2, NULL, global_item_size, local_item_size, 0, NULL, NULL);

            //Read result from gpu
            ret = clEnqueueReadBuffer(*queue, stuff[4], CL_TRUE, 0, sizeof(int), &result, 0, NULL, NULL);

            clFinish(*queue);

            important->zzzzz++;

            if (result != 0) {
                printf("poggerss%d\n", important->zzzzz);
                slidekick_tenk_stuff(m->faceAngle, m->forwardVel, x2, y2, z2, important, secondfall);
            }
            break;
        }
        continue;

        }
        continue;
    }
}

void gpu_slidekick_start(signed short angle, FILE *secondfall) {

    float qspeed;
    int a;
    float finalspeed;

    struct importantshit importantstuff;
    struct importantshit *important = &importantstuff;

    important->zzzzz = 1;

    cl_command_queue queue;
    cl_kernel kernel;
    cl_mem stuff[5];

    gpu_setup(&queue, &kernel, stuff);

    //elevator1
    for (a = 0; a < 4593; a++) {
        if ((signed short) elevator1[(a * 4) + 3] != angle) {
            continue;
        }

        qspeed = elevator1[(a * 4)];
        finalspeed = elevator1[(a * 4) + 2];

        if (qspeed <= 2097152) {
            while (qspeed <= finalspeed) {
                gpu_slidekick_crouch_slide(qspeed * 4, angle, important, secondfall, &queue, &kernel, stuff);
                qspeed = qspeed + 0.25;
            }
        } else {
            while (qspeed <= finalspeed) {
                gpu_slidekick_crouch_slide(qspeed * 4, angle, important, secondfall, &queue, &kernel, stuff);
                qspeed = nextafterf(qspeed, 340282346638528859811704183484516925440.0f);
            }
        }


    }

    //elevator2
    for (a = 0; a < 3548; a++) {
        if ((signed short) elevator2[(a * 4) + 3] != angle) {
            continue;
        }

        qspeed = elevator2[(a * 4)];
        finalspeed = elevator2[(a * 4) + 2];

        if (qspeed <= 2097152) {
            while (qspeed <= finalspeed) {
                gpu_slidekick_crouch_slide(qspeed * 4, angle, important, secondfall, &queue, &kernel, stuff);
                qspeed = qspeed + 0.25;
            }
        } else {
            while (qspeed <= finalspeed) {
                gpu_slidekick_crouch_slide(qspeed * 4, angle, important, secondfall, &queue, &kernel, stuff);
                qspeed = nextafterf(qspeed, 340282346638528859811704183484516925440.0f);
            }
        }
    }
}