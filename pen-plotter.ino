
/* Pen cnc plotter control code
 * (C) depletionmode <David Kaplan>, 2016
 *
 * Licensed under the GPLv3 (see text at EOF)
 *
 * Compiles for Particle (nee Spark) Core FW v0.3.4
 *
 * Some code derived from https://github.com/MarginallyClever/GcodeCNCDemo used under GPLv3 - thanks!
 * Inspired by http://www.ardumotive.com/new-cnc-plotter.html - thanks!
 */

#include "Stepper/Stepper.h"

#define Z_STEPS_PER_POINT 20
#define Y_STEPS_PER_POINT 20
#define X_ROTATIONS_PER_POINT 20
#define X_DUTY_CYCLE 0.20
#define X_MAX 40
#define Y_MAX 40

Stepper _stepper_Z(20, D0, D1, D2, D3);
Stepper _stepper_Y(20, D4, D5, D6, D7);

int _current_coord_X;
int _current_coord_Y;

int _mode_absolute;

int _current_rotary_probe;

void X_move(int pts) {
    if (_current_coord_X + pts < 0) {
        // move to 0
        pts = _current_coord_X * -1;
    } else if (_current_coord_X + pts > X_MAX -1)  {
        // move to X_MAX
        pts = X_MAX - _current_coord_X;
    }
    
    _current_coord_X += pts;
    
    // X-axis is driven by a DC motor and rotary encoder
    int fwd = pts > 0 ? 1 : 0;
    if (!fwd) pts *= -1;    // abs(pts)
    while (pts--) {
        // run motor, counting rotations until a single point moved
        // todo - pwm
        analogWrite(fwd ? A0 : A1, (int)(X_DUTY_CYCLE / 256.0));
        int i = X_ROTATIONS_PER_POINT;
        while (i) {
            if (digitalRead(A2) != _current_rotary_probe) {
                _current_rotary_probe ^= 1;
                i--;
            }
        }
        analogWrite(fwd ? A0 : A1, 0);
    }
}

void Y_move(int pts) {
    if (_current_coord_Y + pts < 0) {
        // move to 0
        pts = _current_coord_Y * -1;
    } else if (_current_coord_Y + pts > Y_MAX -1)  {
        // move to Y_MAX
        pts = Y_MAX - _current_coord_Y;
    }
    
    _current_coord_Y += pts;
    
    _stepper_Y.step(pts);
}

void Z_up() {
    _stepper_Z.step(-10);
}

void Z_down() {
    _stepper_Z.step(10);
}

int adjust_pen(String direction) {
    _stepper_Z.step(direction == "down" ? 1 : -1);
    
    return direction == "down" ? 0 : 1;
}

/* derived from https://github.com/MarginallyClever/GcodeCNCDemo/blob/master/GcodeCNCDemo2Axis/GcodeCNCDemo2Axis.ino */
void draw_line(int X, int Y) {
    int dx = X - _current_coord_X;
    int dy = Y - _current_coord_X;
  
    int dirx = dx > 0 ? 1 : -1;
    int diry = dy > 0 ? 1 : -1; 
  
    dx = abs(dx);
    dy = abs(dy);

    if (dx > dy) {
        int over = 0;
        for (int i=0; i < dx; i++) {
            X_move(dirx);
            
            over += dy;
            if (over >= dx) {
                over -= dx;
                Y_move(diry);
            }
            delay(100);
        }
    } else {
        int over = 0;
        for (int i=0; i < dy; i++) {
            Y_move(diry);
            
            over += dx;
            if (over >= dy) {
                over -= dy;
                X_move(dirx);
            }
            delay(100);
        }
    }
}

int parse_gcode(String gcode) {
    // hack to remove leading 0 in command to support 2-digit format
    //gcode = gcode.toUpperCase().replace("G0", "G");
    
    // pen up/down
    if (gcode.startsWith("M300")) {
        if (gcode.indexOf("S30") > -1)
            Z_down();
        else if (gcode.startsWith("S50"))
            Z_up();
        return 300;
    }
    
    // move
    if (gcode.startsWith("G0") ||
        gcode.startsWith("G1")) {
            int x_str_pos = gcode.indexOf("X");
            int y_str_pos = gcode.indexOf("Y");
            
            int X = _mode_absolute ? 0 : _current_coord_X;
            int Y = _mode_absolute ? 0 : _current_coord_Y;
            
            if (x_str_pos > 0) {
                int ws_pos = gcode.indexOf(".", x_str_pos);
                
                X += gcode.substring(x_str_pos + 1, x_str_pos + ws_pos).toInt();
            }
            
            if (y_str_pos > 0) {
                int ws_pos = gcode.indexOf(".", y_str_pos);
                
                Y += gcode.substring(y_str_pos + 1, y_str_pos + ws_pos).toInt();
            }
            
            draw_line(X, Y);
            return 1;
    }
    
    // puase
    if (gcode.startsWith("G4")) {
        int val_pos = gcode.indexOf("P");
        int ws_pos = gcode.indexOf(" ", val_pos);
        delay(1000 * gcode.substring(val_pos + 1, val_pos + ws_pos).toInt());
        return 4;
    }
    
    // set absolute mode
    if (gcode.startsWith("G90")) {
        _mode_absolute = 1;
        return 90;
    }
    
    // set relative mode
    if (gcode.startsWith("G91")) {
        _mode_absolute = 0;
        return 91;
    }
    
    // set logical position
    if (gcode.startsWith("G92")) {
            int x_str_pos = gcode.indexOf("X");
            int y_str_pos = gcode.indexOf("Y");
            
            if (x_str_pos > 0) {
                int ws_pos = gcode.indexOf(" ", x_str_pos);
                _current_coord_X = gcode.substring(x_str_pos, x_str_pos + ws_pos).toInt();
            }
            
            if (y_str_pos > 0) {
                int ws_pos = gcode.indexOf(" ", y_str_pos);
                _current_coord_Y = gcode.substring(y_str_pos, y_str_pos + ws_pos).toInt();
            }
            
            return 92;
    }
    
    return -1;
}

void setup() {
    _current_coord_X = 0;
    _current_coord_Y = 0;
    _mode_absolute = 0;
    
    pinMode(A0, OUTPUT);    // X-axis FWD
    pinMode(A1, OUTPUT);    // X-axis REV
    pinMode(A2, INPUT);     // X-axis optical rotary encoder
    _current_rotary_probe = digitalRead(A2);   // read current value of rotary encoder
    
    _stepper_Z.setSpeed(100);
    _stepper_Y.setSpeed(100);

    Spark.function("gcode", parse_gcode);
    Spark.function("adjust_pen", adjust_pen);
}

void loop() { /* ... waiting ... waiting ... waiting ... */ }

/**
* This file is part of pen-plotter by depletionmode.
*
* GcodeCNCDemo is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GcodeCNCDemo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with pen-plotter. If not, see <http://www.gnu.org/licenses/>.
*/

