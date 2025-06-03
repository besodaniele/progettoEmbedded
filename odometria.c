#include <math.h>
#include <stdio.h>
#include <linux/types.h>

#define BASELINE 120.0 // distanza tra le ruote in mm

// funzione di moltiplicazione matrici 4x4
void prod_matrix_4_4(double res[4][4], double A[4][4], double B[4][4]) {
    double temp[4][4];
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j) {
            temp[i][j] = 0;
            for(int k = 0; k < 4; ++k)
                temp[i][j] += A[i][k] * B[k][j];
        }

    // Copia in res
    for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
            res[i][j] = temp[i][j];
}

// aggiorna la pose usando i tick encoder
void update_pose_from_ticks(double pose[4][4], int delta_left_ticks, 
  int delta_right_ticks, double mm_per_tick_left, double mm_per_tick_right,
  double BASELINE) {

    // Calcola la distanza percorsa da ciascuna ruota in mm
    double dL = delta_left_ticks * mm_per_tick_left;
    double dR = delta_right_ticks * mm_per_tick_right;

    // Calcola la distanza percorsa dal centro del robot
    double d = (dL + dR) / 2.0;

    // Calcola la variazione dell'angolo di orientamento (in radianti)
    // in base alla differenza tra le due ruote
    double delta_theta = (dR - dL) / BASELINE;

    // Costruisce la matrice omogenea 4x4 di roto-traslazione T
    double T[4][4] = {
        {cos(delta_theta), -sin(delta_theta), 0, d},
        {sin(delta_theta),  cos(delta_theta), 0, 0},
        {0,                 0,                1, 0},
        {0,                 0,                0, 1}
    };

    prod_matrix_4_4(pose, pose, T);
}

// stampa la posizione del robot + angoli RPY e ZYZ
void print_pose_info(double pose[4][4]) {
    double x = pose[0][3];
    double y = pose[1][3];
    double z = pose[2][3];

    printf("\nPosizione → x = %.2f mm, y = %.2f mm, z = %.2f mm\n", x, y, z);

    // Angoli ZYZ (Euler)
    double theta_z1 = atan2(pose[1][2], pose[0][2]);
    double theta_y  = atan2(sqrt(pow(pose[0][2],2)+pow(pose[1][2],2)), pose[2][2]);
    double theta_z2 = atan2(pose[2][1], -pose[2][0]);

    printf("ZYZ Euler (°): Z1 = %.2f, Y = %.2f, Z2 = %.2f\n",
           theta_z1 * 180/M_PI, theta_y * 180/M_PI, theta_z2 * 180/M_PI);

    // Angoli RPY (Roll-Pitch-Yaw)
    double roll  = atan2(pose[1][0], pose[0][0]);
    double pitch = atan2(-pose[2][0], sqrt(pow(pose[2][1],2)+pow(pose[2][2],2)));
    double yaw   = atan2(pose[2][1], pose[2][2]);

    printf("RPY (°): Roll = %.2f, Pitch = %.2f, Yaw = %.2f\n\n",
           roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI);
}


//
// da inserire nella tua funzione start()

// matrice pose che indica la nuova posizione del robot ad
// ogni iterazione, secondo il sistema cartesiano body
// da inserire dopo aver dichiarato gli encoder
  double pose[4][4] = {
    {1, 0, 0, 0}, // R | t
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}  // omogeneità
  };

// da mettere nel ciclo for dentro a start()
    // Calcola differenza di tick dall'iterazione precedente
    __i64 delta_left = encoder_left.ticks - left_prev_ticks;
    __i64 delta_right = encoder_right.ticks - right_prev_ticks;
    left_prev_ticks = encoder_left.ticks;
    right_prev_ticks = encoder_right.ticks;

    // aggiorna la pose tramite tick encoder -- odometria
    update_pose_from_ticks(pose, delta_left, delta_right,
                       MillimeterFromTicks_Left,
                       MillimeterFromTicks_Right,
                       BASELINE);

    // stampa posizione e angoli
    print_pose_info(pose);

