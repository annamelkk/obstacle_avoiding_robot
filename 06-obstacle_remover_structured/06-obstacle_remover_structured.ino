#include "robot.h"

unsigned long last_sonar = 0;
int DISTANCE = 999;

void setup()
{
    Serial.begin(9600);

    setup_pins();

    stop();
    neutral_position();

    delay(1000);
    last_pid_time = millis();
}

void loop()
{
    unsigned long now = millis();

    // ===== SERIAL CONTROL =====
    if (Serial.available())
    {
        char c = Serial.read();

        if (c == '0')
        {
            current_mode = ON_OFF_MODE;
            Serial.println("Controller: ON/OFF");
        }
        else if (c == '1')
        {
            current_mode = P_MODE;
            Serial.println("Controller: P");
        }
        else if (c == '2')
        {
            current_mode = PID_MODE;
            Serial.println("Controller: PID");
        }
    }

    // ===== SONAR =====
    if (now - last_sonar > SONAR_INTERVAL)
    {
        last_sonar = now;
        DISTANCE = get_distance();
    }

    // ===== OBSTACLE =====
    if (DISTANCE < OBSTACLE_DISTANCE)
    {
        stop();
        tone(BUZZER, 1000);
        delay(300);
        noTone(BUZZER);

        grab_bottle();
        remove_bottle();

        delay(100);

        last_pid_time = millis();
        integral = 0;

        return;
    }

    // ===== LINE FOLLOWING =====
    int ldr = analogRead(LDR);
    run_controller(ldr); //
}
