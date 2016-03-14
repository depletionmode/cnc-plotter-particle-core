
/* Pen cnc plotter control code for Particle.io Core
 * (C) depletionmode <David Kaplan>, 2016
 *
 * Licensed under the GPLv3 (see text at EOF)
 *
 * Some code derived from https://github.com/MarginallyClever/pen-plotter used under GPLv3 - thanks!
 * Inspired by http://www.ardumotive.com/new-cnc-plotter.html - thanks!
 */
 
//SYSTEM_MODE(SEMI_AUTOMATIC);

#include "Stepper.h"

#define X_LENGTH_PTS 105
#define Y_LENGTH_PTS 105

#define Y_STEPS_PER_POINT 2
#define X_MS_PER_POINT 6

#define Y_STEPPER_A1_PIN A2
#define Y_STEPPER_A2_PIN A4
#define Y_STEPPER_B1_PIN A1
#define Y_STEPPER_B2_PIN A5

#define X_SERVO_PIN A7
#define Z_SERVO_PIN A0

#define LED_BUSY D6
#define LED_IDLE D7

Stepper _stepper_Y(20, Y_STEPPER_A1_PIN, Y_STEPPER_A2_PIN, Y_STEPPER_B1_PIN, Y_STEPPER_B2_PIN);

int _current_coord_X;
int _current_coord_Y;

int _mode_absolute;

int _current_rotary_probe;

void X_move(int pts) {
    if (_current_coord_X + pts < 0) {
        // move to 0
        pts = _current_coord_X * -1;
    } else if (_current_coord_X + pts > X_LENGTH_PTS -1)  {
        // move to X_LENGTH_PTS
        pts = X_LENGTH_PTS - _current_coord_X;
    }
    
    _current_coord_X += pts;
    
    int fwd = pts > 0 ? 1 : 0;
    if (!fwd) pts *= -1;    // abs(pts)
    analogWrite(X_SERVO_PIN, fwd ? 250 : 10);
    delay(X_MS_PER_POINT*pts * (fwd ? 0.9 : 0.875));
    analogWrite(X_SERVO_PIN, 0);
    //Serial.println(X_MS_PER_POINT*pts);
}

void Y_move(int pts) {
    if (_current_coord_Y + pts < 0) {
        // move to 0
        pts = _current_coord_Y * -1;
    } else if (_current_coord_Y + pts > Y_LENGTH_PTS -1)  {
        // move to Y_LENGTH_PTS
        pts = Y_LENGTH_PTS - _current_coord_Y;
    }
    
    _current_coord_Y += pts;
    
    _stepper_Y.step(pts*Y_STEPS_PER_POINT);
}

void Z_up() {
    analogWrite(Z_SERVO_PIN, 50);
    delay(40);
    analogWrite(Z_SERVO_PIN, 0);
}

void Z_down() {
    analogWrite(Z_SERVO_PIN, 250);
    delay(38);
    analogWrite(Z_SERVO_PIN, 0);
}

int adjust_axis(String axis) {
    if (axis == "X") {
        analogWrite(X_SERVO_PIN, 10);
        delay(X_MS_PER_POINT*5);
        analogWrite(X_SERVO_PIN, 0);
    } else if (axis == "reset") {
        draw_line(0, 0);
    } else {
        _stepper_Y.step(-1*Y_STEPS_PER_POINT*5);
    }
    
    return axis == "X" ? 0 : 1;
}


int adjust_pen(String direction) {
    if (direction == "down") {
        analogWrite(Z_SERVO_PIN, 250);
        delay(7);
        analogWrite(Z_SERVO_PIN, 0);
    } else {
        analogWrite(Z_SERVO_PIN, 50);
        delay(7);
        analogWrite(Z_SERVO_PIN, 0);
    }
    
    return direction == "down" ? 0 : 1;
}

/* derived from https://github.com/MarginallyClever/pen-plotter/blob/master/pen-plotter2Axis/pen-plotter2Axis.ino */
void draw_line(int X, int Y) {
    digitalWrite(LED_BUSY, LOW);
    digitalWrite(LED_IDLE, HIGH);
    
    Serial.println("X");
    Serial.println(Y);
    Serial.println("Y");
    Serial.println(X);
    // convert mm to pts
    /*X *= PTS_PER_MM;
    Y *= PTS_PER_MM;*/
    
    int dx = X - _current_coord_X;
    int dy = Y - _current_coord_Y;
  
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
    
    digitalWrite(LED_BUSY, HIGH);
    digitalWrite(LED_IDLE, LOW);
}

void draw_arc(float cx,float cy,float x,float y,float dir) {
	// todo
}

int parse_gcode(String gcode) {
    int cnt = 0;
    
    // tokenize
    while (gcode.length() > 0) {
        cnt++;
        
        int delim_at = gcode.indexOf("\n");
        if (delim_at >= 0) {
            parse_gcode_line(gcode.substring(0, delim_at));
            gcode = gcode.substring(delim_at + 1);
        } else {
            // final command may not be '\n' terminated
            parse_gcode_line(gcode);
            break;
        }
    }
    
    return cnt;
}


int parse_gcode_line(String gcode) {
    Serial.print(gcode + "\n");
    
    // hack to remove leading 0 in command to support 2-digit format
    gcode = gcode.toUpperCase().replace("G0", "G");
    
    // pen up/down
    if (gcode.startsWith("M300")) {
        if (gcode.indexOf("S30") > -1)
            Z_down();
        else if (gcode.indexOf("S50") > -1)
            Z_up();
        return 300;
    }
    
    // move - line
    if (gcode.startsWith("G0") ||
        gcode.startsWith("G1")) {
            int x_str_pos = gcode.indexOf("X");
            int y_str_pos = gcode.indexOf("Y");
            
            int X = _mode_absolute ? 0 : _current_coord_X;
            int Y = _mode_absolute ? 0 : _current_coord_Y;
            
            if (x_str_pos > 0) {
                int ws_pos = gcode.indexOf(".", x_str_pos);
                
                X += gcode.substring(x_str_pos + 1, x_str_pos + ws_pos).toInt();
            } else
                X = _current_coord_X;
            
            if (y_str_pos > 0) {
                int ws_pos = gcode.indexOf(".", y_str_pos);
                
                Y += gcode.substring(y_str_pos + 1, y_str_pos + ws_pos).toInt();
            } else
                Y = _current_coord_Y;
            
            draw_line(X, Y);
            return 
            1;
    }
    
    // move - arc
    if (gcode.startsWith("G2") ||
        gcode.startsWith("G3")) {
            int x_str_pos = gcode.indexOf("X");
            int y_str_pos = gcode.indexOf("Y");
            int i_str_pos = gcode.indexOf("I");
            int j_str_pos = gcode.indexOf("J");
            
            int X = _mode_absolute ? 0 : _current_coord_X;
            int Y = _mode_absolute ? 0 : _current_coord_Y;
            int I = 0;
            int J = 0;
            
            if (x_str_pos > 0) {
                int ws_pos = gcode.indexOf(".", x_str_pos);
                
                X += gcode.substring(x_str_pos + 1, x_str_pos + ws_pos).toInt();
            }
            
            if (y_str_pos > 0) {
                int ws_pos = gcode.indexOf(".", y_str_pos);
                
                Y += gcode.substring(y_str_pos + 1, y_str_pos + ws_pos).toInt();
            }
            
            if (i_str_pos > 0) {
                int ws_pos = gcode.indexOf(".", i_str_pos);
                
                I += gcode.substring(i_str_pos + 1, i_str_pos + ws_pos).toInt();
            }
            
            if (j_str_pos > 0) {
                int ws_pos = gcode.indexOf(".", j_str_pos);
                
                J += gcode.substring(j_str_pos + 1, j_str_pos + ws_pos).toInt();
            }
            
            draw_arc(I, J, X, Y, gcode.startsWith("G2") ? -1 : 1);
            return 1;
    }
    
    // pause
    if (gcode.startsWith("G4")) {
        int val_pos = gcode.indexOf("P");
        int ws_pos = gcode.indexOf(" ", val_pos);
        delay(1000 * gcode.substring(val_pos + 1, val_pos + ws_pos).toInt());
        return 4;
    }
    
    // move to origin
    if (gcode.startsWith("G28")) {
        Z_up();
        
        // todo - fix
        draw_line(-1*_current_coord_X, -1*_current_coord_Y);
        
        return 28;
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

int count;
void setup() {
    Serial.begin(9600);
    Serial.print("CNC Plotter by depletionmode");

    _current_coord_Y = 0;
    _mode_absolute = 1;

    pinMode(X_SERVO_PIN, OUTPUT);
    pinMode(Z_SERVO_PIN, OUTPUT);

    pinMode(LED_BUSY, OUTPUT);
    pinMode(LED_IDLE, OUTPUT);
    digitalWrite(LED_BUSY, HIGH);
    digitalWrite(LED_IDLE, LOW);

    _stepper_Y.setSpeed(100);

    Particle.function("gcode", parse_gcode);
    Particle.function("adjustPen", adjust_pen);
    Particle.function("adjustAxis", adjust_axis);
}

void loop() { /* ... waiting ... waiting ... waiting ... */ }

/**
* This file is part of pen-plotter by depletionmode.
*
* pen-plotter is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* pen-plotter is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with pen-plotter. If not, see <http://www.gnu.org/licenses/>.
*/
