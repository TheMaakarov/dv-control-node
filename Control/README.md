# Control v0.9.1

## Index

- [TODO](#todo)
- [Instalación](#instalación)
  - [Descargar repositorio](#descargar-repositorio)
- [Configuración previa](#configuración)
- [Ejecución](#ejecución)

----
## TODO:
- [ ] Verificar inputs provenientes del nodo de _PathPlanning_.
- [ ] Resolver problema con orientación inicial.
- [ ] Parametrizar en un archivo de configuración las variables relativas al modelo del vehículo.
- [ ] Corregir posibles errores y pulir _warnings_.

----
Dependiendo del entorno en el que se esté (un entorno habitual de Ubuntu, ya sea máquina virtual o WSL ; o bien en el entorno del ordenador de a bordo), se especificará **TEST** o **CUBO**, cuando se precise una especificación.

----
## Instalación:
- Para descargar el código desde Github es necesario ser miembro de la organización [Uvigo-Motorsport](https://github.com/Uvigo-Motorsport).

- Para ello basta con notificarlo al equipo. Será necesaria una cuenta de github, idealmente utilizando el correo de la Universidad, aunque no es excesivamente relevante.

- A continuación, es necesario configurar Git en local para cumplir con los permisos. Esto se puede hacer siguiendo [este tutorial](https://www.freecodecamp.org/news/git-ssh-how-to/).

> Será necesario configurar un par de claves para cada equipo con el que se desee acceder a los repositorios.

- Además, si no se ha hecho antes, se debe configurar Git de la siguiente manera:
```
git config user.name “<github_username>”
git config user.email “<github_email>”
```

### Descargar repositorio
- Para clonar el repositorio, ir a la carpeta de inicio y usar:
```
git clone https://github.com/Uvigo-Motorsport/Control.git
```
> En el cubo, se necesita que la carpeta de descarga sea `~/git/`, y no `~/` a secas.
> En caso de fallo, es probable que sea necesario configurar unas claves SSH en local y configurarlas en GitHub. Dada la duda, preguntar.

- Una vez descargado el repositorio, se habrá creado la carpeta Control en el directorio actual. Este será nuestro workspace. Acceder a la carpeta y compilar:
```
cd Control
catkin_make
source ~/.bashrc
```
A continuación:

**CUBO**:
```
source ~/git/Control/devel/setup.bash
```
**TEST**:
```
source ~/Control/devel/setup.bash
``` 
> Es recomendable añadir esta última línea al final de `~/.bashrc` para agilizar futuras ejecuciones.

---
## Configuración

### Configuración (CUBO)
Dentro del archivo *lib/PythonRobotics/environment.yml* vienen definidas unas librerías a descargar para un entorno de ejecución de Python3.9.
Para instalarlo en el Cubo, como Anaconda da problemas (de momento), habrá que hacerlo de forma manual.
Para ello, bastaría con:
```
pip3.9 install scipy numpy pandas cvxpy matplotlib
```

### Configurar Anaconda (TEST)
Para configurar el entorno de Anaconda para Python 3 se incluye el archivo de configuración de entorno _environment.yml_. Para cargarlo:
```
conda env create -f lib/PythonRobotics/environment.yml
```

### SI SE ESTÁ USANDO WSL:
Para el visionado gráfico de la ejecución, es necesario instalar VcXsrv. Para ello, se puede seguir la respuesta a [este enlace](https://stackoverflow.com/questions/43397162/show-matplotlib-plots-and-other-gui-in-ubuntu-wsl1-wsl2) de Stack Overflow.
En resumen, se debe instalar [VcXsrv](https://sourceforge.net/projects/vcxsrv) y lanzarlo con la configuración que se muestra en la página y configurar un display para el entorno de Linux añadiendo la siguiente línea al final de `~/.bashrc`:
```
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

Y cargarlo con:
```
source ~/.bashrc
```

----
## Ejecución:
Se necesitarán tres pestañas, en el orden especificado.

- Pestaña 1 (RosCore)
  - Lanzar el servicio RosCore:
  ```
  roscore
  ```

- Pestaña 2 (MPC en Python 3.9)
  - Activar entorno y lanzar el programa:
  
  **CUBO**
  ```
  python3.9 ~/git/Control/src/control/scripts/mpssc.py
  ```
  **TEST**
  ```
  conda activate python_robotics
  python ~/Control/src/control/scripts/mpssc.py
  ```

- Pestaña 3 (Nodo de Control)
  - Ejecutar el nodo:
  ```
  rosrun control sender.py
  ```

El sistema debería comenzar a funcionar y, por pantalla (en la pestaña 3), se deberían ver la Pose actual, aceleración y giro recibidos desde el entorno de Anaconda según se procesan.

A la vez, una ventana de _matplotlib_ debería mostrar el vehículo en cuestión trazando el Spline iteración tras iteración, si está activado.

