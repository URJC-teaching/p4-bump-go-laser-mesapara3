# p4_bump_go_laser

El objetivo de esta prráctica es hacer que el robot navegue sin colisionar con los obstáculos del entorno. Para ello:

- Se utilizará una máquina de estados que definirá el comportamiento al encontrar un obstáculo cercano.
- Se utilizará el laser como sensor para detectar los obstáculos antes de colisionar.
- Se transformarán los puntos del laser al frame base_link mediante la transformada proporcionada por tf
