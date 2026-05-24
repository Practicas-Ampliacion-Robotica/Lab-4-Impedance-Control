# Lab-4-Impedance Control

## Introducción 
El objetivo de esta práctica es diseñar e implementar un controlador de impedancia controlar y manipulador de dos grados de libertad 
con ayuda de ROS2.

Posteriormente, se cambiarán los parámetros de las impedancias para ver como se comporta el manipulador al aplicarle diferentes
fuerzas externas y se arreglarar el problema del acoplamiento cruzado que se produce en el robot. 

Finalmente, se cambiará la posición deseada del robot para ver como se comporta el sistema. 

## Fundamentos teóricos

Para diseñar el controlador de impedancias se aplicará el siguiente diagrama de bloques con el cual se produce el control de impedancias. 

<img src="images/diagram_bloques.png" width="50%">

El esquema está formado por:

- Forward Kinematics: se encarga de optener la posición del EE en el espacio cartesiano a partir de de las coordenadas articulares ($$q$$). Para eso
se aplica la siguiente fórmula:

$$\mathbf{x} = 
\begin{bmatrix}
l_1\cos(q_1) + l_2\cos(q_1 + q_2) \\
l_1\sin(q_1) + l_2\sin(q_1 + q_2)
\end{bmatrix}$$

- Differential Kinematics ($$J(q)$$): se escaga de obtener las velocidades cartesianas a partir de las velocidades articulares.
Se aplica la siguiente fórmula:

$$\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q})\dot{\mathbf{q}}$$

- Control de impedancia: es este bloque se realiza el control de impedancia del manipulador. Para ello, se le dice a la cadena cinemática la
aceleración deseada a partir de los datos recibidos de los sensores. A partir del modelo de impedancia de segundo orden:

$$\mathbf{M}\ddot{\tilde{\mathbf{x}}} + \mathbf{B}\dot{\tilde{\mathbf{x}}} + \mathbf{K}\tilde{\mathbf{x}} = \mathbf{f}_{ext}$$

Se obtienen las aceleraciones deseadas:

$$\ddot{\mathbf{x}}_d = \mathbf{M}^{-1}(\mathbf{f}_{ext} - \mathbf{B}\dot{\tilde{\mathbf{x}}} - \mathbf{K}\tilde{\mathbf{x}})$$

Donde: $\tilde{\mathbf{x}} = \mathbf{x} - \mathbf{x}_d$ y $\dot{\tilde{\mathbf{x}}} = \dot{\mathbf{x}} - \dot{\mathbf{x}}_d$.

- Aceleraciones deseadas: pasa las aceleraciones del espacio cartesiano al articular. Para ello, se implementa la inversea de la ecuación
de la cinemática diferencial de segundo orden:

$$\ddot{\mathbf{x}} = \mathbf{J}(\mathbf{q})\ddot{\mathbf{q}} + \dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$$

$$\ddot{\mathbf{q}} = \mathbf{J}(\mathbf{q})^{-1}[\ddot{\mathbf{x}} - \dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}]$$


## Código implementado

Para calcular estas fórmulas, es necesario calcular anteriormente las matrices jacobianas. Para ello, se implementa la función $$Jacobians Update$$.

Es ella se calculan las matrices jacobianas de la posición y la velocidad:

$$\mathbf{J}(\mathbf{q}) = 
\begin{bmatrix}
-l_1\sin(q_1) - l_2\sin(q_1 + q_2) & -l_2\sin(q_1 + q_2) \\
l_1\cos(q_1) + l_2\cos(q_1 + q_2) & l_2\cos(q_1 + q_2)
\end{bmatrix}$$

$$\dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}}) = 
\begin{bmatrix}
-l_1\cos(q_1)\dot{q}_1 - l_2\cos(q_1 + q_2)(\dot{q}_1 + \dot{q}_2) & -l_2\cos(q_1 + q_2)\dot{q}_2 \\
-l_1\sin(q_1)\dot{q}_1 - l_2\sin(q_1 + q_2)(\dot{q}_1 + \dot{q}_2) & -l_2\sin(q_1 + q_2)\dot{q}_2
\end{bmatrix}$$

En el siguiente código:
```cpp
// Method to update jacobian and jacobian derivative
void update_jacobians()
{
    // Declaramos las variables locales a partir de los atributos de la clase
    double q1 = joint_positions_(0);
    double q2 = joint_positions_(1);
    double q_dot1 = joint_velocities_(0);
    double q_dot2 = joint_velocities_(1);
    // Placeholder for jacobian and jacobian_derivative matrices

    // Calculate J(q)
    jacobian_ << (-l1_ * std::sin(q1)) - (l2_ * std::sin(q1 + q2)), -l2_ * std::sin(q1 + q2),
             (l1_ * std::cos(q1)) + (l2_ * std::cos(q1 + q2)),  l2_ * std::cos(q1 + q2);

    // Calculate J'(q,q')
    jacobian_derivative_ << (-l1_ * std::cos(q1) * q_dot1) - (l2_ * std::cos(q1 + q2) * q_dot1), -l2_ * std::cos(q1 + q2) * q_dot2,
                        (-l1_ * std::sin(q1) * q_dot1) - (l2_ * std::sin(q1 + q2) * q_dot1), -l2_ * std::sin(q1 + q2) * q_dot2;

    RCLCPP_INFO(this->get_logger(), "Jacobian:\n[%.3f, %.3f]\n[%.3f, %.3f]",
                jacobian_(0, 0), jacobian_(0, 1),
                jacobian_(1, 0), jacobian_(1, 1));

    double det = jacobian_.determinant();
    RCLCPP_INFO(this->get_logger(), "Jacobian determinant: %.6f", det);
}

```

Posteriormente, las funciones mencionadas anteriormente, está implementadas de la siguiente forma:

- differential_kinematics:

```cpp
// Method to calculate Cartesian velocity with the first-order differential kinematics
Eigen::MatrixXd differential_kinematics()
{
    // Placeholder for first-order differential kinematics
    Eigen::VectorXd x_dot(2);
    x_dot = jacobian_ * joint_velocities_; // antes había <<

    return x_dot;
}
```

- forward_kinematics

```cpp
// Method to calculate forward kinematics
Eigen::VectorXd forward_kinematics()
{
    // Declaramos las variables locales a partir de los atributos de la clase
    double q1 = joint_positions_(0);
    double q2 = joint_positions_(1);

    // Placeholder for forward kinematics x = [l1_ * cos(q1) + l2_ * cos(q1 + q2), l1_ * sin(q1) + l2_ * sin(q1 + q2)]
    Eigen::VectorXd x(2);
    x << (l1_ * std::cos(q1)) + (l2_ * std::cos(q1 + q2)),
         (l1_ * std::sin(q1)) + (l2_ * std::sin(q1 + q2));

    return x;
}
```

- impedance_controller

```cpp
// Method to compute the impedance controller
Eigen::VectorXd impedance_controller()
{
    // Placeholder for impedance controller calculation
    Eigen::VectorXd x_dot_d = Eigen::VectorXd::Zero(2); // We assume desired cartesian velocity = 0

    // Calculate Cartesian errors
    Eigen::VectorXd x_error(2);
    x_error = cartesian_pose_ - equilibrium_pose_;
    Eigen::VectorXd x_dot_error(2);
    x_dot_error = cartesian_velocities_ - x_dot_d;

    // Replace with actual impedance controller equation: x'' = M^(-1)[F_ext - k x_error - B x'_error]
    Eigen::VectorXd x_ddot(2);
    x_ddot = mass_matrix_.inverse() * (external_wrenches_ - (damping_matrix_ * x_dot_error) - (stiffness_matrix_ * x_error));
    // antes era <<
    return x_ddot;
}
```

- calculate_desired_joint_accelerations

```cpp
// Method to calculate joint acceleration with the inverse of second-order differential kinematics
Eigen::VectorXd calculate_desired_joint_accelerations()
{
    // Placeholder for the second-order differential kinematics
    // q'' = J(q)^(-1)[x'' - J'(q,q')q']

    RCLCPP_INFO(this->get_logger(), "x_ddot: [%.3f, %.3f]",
                desired_cartesian_accelerations_(0), desired_cartesian_accelerations_(1));

    Eigen::VectorXd q_ddot = jacobian_.inverse() * (desired_cartesian_accelerations_ - (jacobian_derivative_ * joint_velocities_));;

    return q_ddot;
}
```


Al lanzar los código se puede ver las siguintes iterración entre los bloques:

<img src="images/rqt_4.3.png" width="100%">

Se puede observar como el programa con el cual se simulan las fuerzas externas (/wrench_trackbar_publisher), pasa las fuerzas 
al  a través del topic /external_wrenches al /impedance_controller, el cual calcula las velocidades deseadas. Al bloque /impedance_controller
también le entran las velocidades y posiciones articulares, las cuales se pasan al espacio cartesiano dentro del bloque. Dentro del bloque
también se pasan las aceleraciones deseadas el espacioi cartesiano al articular. 

Esas aceleraciones van al bloque de /dynamics_cancellation, donde se implementó la compensación  dinámica en anteriores prácticas. Las 
aceleraciones se pasan por el topic /desired_joint_accelerations. 

Con el bloque de /dynamics_cancellation, se calculan los torques que debe aplicar cada articulación para estar en una posición. Esos torques
pasan a través del topic /joint_torques al bloque /uma_arm_dynamics que calcula las posción, velocidad y aceleración de cada articulación 
tras aplicarle los torques interno. Esos parámetros se llevan al resto de bloques para ser representados en la simulación y ser usados
en el resto de bloques. 



## Resultados

Se han realizado varias pruebas con diferentes parámetros para ver cómo se comporta el manipulador en cada caso.

### Prueba con parámetros por defecto

Para esta prueba se han usado los siguientes parámetros:
*   `M`: [1.0, 0.0, 0.0, 1.0] 
*   `B`: [100.0, 0.0, 0.0, 100.0]
*   `K`: [250.0, 0.0, 0.0, 250.0]

Se obtienen los siguientes resultados:

- Muestra en vídeo
<img src="images/gif_4.3_base.gif" width="50%">


- Muestra gráfica
<img src="images/yaml_base.png" width="50%">


### Prueba con M = 10

Para esta prueba se han usado los siguientes parámetros:
*   `M`: [10.0, 0.0, 0.0, 10.0] 
*   `B`: [100.0, 0.0, 0.0, 100.0]
*   `K`: [250.0, 0.0, 0.0, 250.0]

Se obtienen los siguientes resultados:

- Muestra en vídeo
<img src="images/gif_4.3_M10.gif" width="50%">


- Muestra gráfica
<img src="images/yaml_m10.png" width="50%">


### Prueba con B = 10

Para esta prueba se han usado los siguientes parámetros:
*   `M`: [1.0, 0.0, 0.0, 1.0] 
*   `B`: [10.0, 0.0, 0.0, 10.0]
*   `K`: [250.0, 0.0, 0.0, 250.0]


Se obtienen los siguientes resultados:

- Muestra en vídeo
<img src="images/gif_4.3_b10.gif" width="50%">


- Muestra gráfica
<img src="images/yaml_b10.png" width="50%">



### Prueba con K = 25

Para esta prueba se han usado los siguientes parámetros:
*   `M`: [1.0, 0.0, 0.0, 1.0] 
*   `B`: [10.0, 0.0, 0.0, 10.0]
*   `K`: [25.0, 0.0, 0.0, 25.0]


Se obtienen los siguientes resultados:

- Muestra en vídeo
<img src="images/gif_4.3_k25.gif" width="50%">


- Muestra gráfica
<img src="images/yaml_k25.png" width="50%">



### Prueba con impedancia en 'X' y 'Y' diferentes

Para esta prueba se han usado los siguientes parámetros:
*   `M`: [1.0, 0.0, 0.0, 1.0] 
*   `B`: [10.0, 0.0, 0.0, 10.0]
*   `K`: [25.0, 0.0, 0.0, 25.0]


Se obtienen los siguientes resultados:

- Muestra en vídeo
<img src="images/gif_4.3_x_h_y_l.gif" width="50%">


- Muestra gráfica
<img src="images/yaml_x_h_y_l.png" width="50%">

















 
