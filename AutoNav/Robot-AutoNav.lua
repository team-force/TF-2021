function sysCall_init()
-- do some initialization here
   
    Robot = sim.getObjectHandle(sim.handle_self)
    Field = sim.getObjectHandle("Field")

    camara = sim.getObjectHandle("Camara_Sensor")
    -- Declarar variables para manipular los joints
    -- Cuales joints necesitamos para mover al robot?
    
    -- Los del centro estan conectados a los motores
    -- Pero todos se mueven al unisono
    
    -- Izquierdos
    MI = sim.getObjectHandle("DriveIzq")
    MIA = sim.getObjectHandle("Joint_IA")
    MID = sim.getObjectHandle("Joint_ID")
    
    MD = sim.getObjectHandle("DriveDer")
    MDA = sim.getObjectHandle("Joint_DA")
    MDD = sim.getObjectHandle("Joint_DD")
    
    v_0 = 0.3
    v_max = 0.50 -- m/s

    D_rueda = 1.522e-1 -- metros -- Medir en CoppeliaSim tamaño de la rueda
    -- Radio de ruedas
    R_rueda = D_rueda/2 --metros
    --Distancia entre ruedas 2*2.84e-1
    L_base = 0.6106 -- metros -- Medir en CoppeliaSim: Distancia entre joints de las ruedas dobles

    pos_inicial = {0,0,0} --sim.getObjectPosition(Robot, Field) -- Posicion inicial del robot en el mapa
    ort_inicial = sim.getObjectOrientation(Robot, Field)

    --- Crea "tablas" o "arrays" con valor de (x,y)
    punto1 = crearPosicion(pulgadasMetros(90),  pulgadasMetros(60)) -- punto D en Geogebra
    punto2 = crearPosicion(pulgadasMetros(270),  pulgadasMetros(60))
    punto3 = crearPosicion(pulgadasMetros(313),  pulgadasMetros(38))
    punto4 = crearPosicion(pulgadasMetros(313),  pulgadasMetros(81))
    punto5 = crearPosicion(pulgadasMetros(30),  pulgadasMetros(90))
    
    radio1 = pulgadasMetros(75)
    radio2 = pulgadasMetros(-112.5)
    radio3 = pulgadasMetros(55.35)
    radio4 = pulgadasMetros(26.22)

   
    v_act = 0
    w_act = 0

    radio_actual = radio1 -- Inicializamos el radio_actual
    print(sim.getJointTargetVelocity(MD))
    print(punto2)
end

-- una funcion que toma una velocidad 
-- y le manda la misma a los otros motores

function sysCall_actuation()
    -- Codigo de actuacion (acción) del robot
    
    pos_actual = leerPosicionRobot()
    
    --[[ Ejercicio: 
    Cambiar el arco según la ubicación del robot.
    1. Usar la imagen del NavChallenge o Geogebra para calcular la ubicación (en metros) X,Y de los puntos de inicio de cada segmento
        - Usar las funciones pos = crearPosicion(x,y) y mtr = pulgadasMetros(in)  para guardar las posiciones (la imagen usa pulgadas)
        - Guardar las posiciones en variables como punto1, punto2, punto3, etc.
    
    2. Usar Geogebra para calcular el Radio de un círculo que una los puntos de inicio de cada segmento.
    
    3. Usar la función arco(v, radio) para mover el robot en cada uno de los radios calculados,
        - Observar si el robot hace el movimiento que se espera.
        - Usar un radio negativo mueve el robot "a la derecha" (con las manecillas del reloj)
    
    --]]

    -- Cambiar el radio_actual segun el punto al queramos llegar:
    -- Para esto, revisamo la cercania del robot al punto en cuestion
    -- Ya pos_actual tiene (x,y) del robot, y punto1 tiene (x,y) del objetivo

    --Cuando esta suficientemente cerca, cambia el radio actual
    print(calcDistancia(pos_actual, punto1))
    if ( calcDistancia(pos_actual, punto1) < 0.07 ) then -- Si el robot esta en menos
       radio_actual = radio2
       print("Radio2")
    elseif (calcDistancia(pos_actual, punto2) < 0.15) then
        radio_actual = radio3
        print("Radio3")
       
    end

end

function sysCall_sensing()
    -- put your sensing code here
    
    --actualizarUI()
end

function sysCall_cleanup()
    -- do some clean-up here
    mover(0,0) --No parece tener ningun efecto
    print("Cerrando")
    print(sim.getJointTargetVelocity(MD))
end

function motorD(vel)
    -- Toma vel y se la pasa a cada motor de la derecha
    sim.setJointTargetVelocity(MD,vel)
    sim.setJointTargetVelocity(MDA,vel)
    sim.setJointTargetVelocity(MDD,vel)
end

function motorI(vel)
    -- Toma vel y se la pasa a cada motor de la derecha
    sim.setJointTargetVelocity(MI,vel)
    sim.setJointTargetVelocity(MIA,vel)
    sim.setJointTargetVelocity(MID,vel)
end


function avanzar(vel)
    mover(vel,vel)
end

function girar(vel)
    mover(vel, -vel)
end

function arcoIzq(vel)
    mover(vel,vel/2)
end

function arcoDer(vel)
    mover(vel/2,vel)
end

function arco(v, radio)
    -- vel de giro depende de la vel de avance y el radio de giro
    w = v/radio --- w -> omega : vel. angular (cambio de direccion)i
    desplazar(v,w)
end

function desplazar(v,w)
    v_act = v
    w_act = w
    -- vel angular de las ruedas
    w_Der = (2*v+L_base*w)/(2*R_rueda) 
    w_Izq = (2*v-L_base*w)/(2*R_rueda)

    mover(w_Der, w_Izq)
end

function mover(velD, velI)

    -- Enviar el comando de velocidad a cada motor
    motorD(velD)
    motorI(velI)
end

function leerPosicionRobot()
    -- Devuelve la posición X,Y del robot con respecto a su punto de inicio
    local d = sim.getObjectPosition(Robot, Field)
    return {d[1]-pos_inicial[1], d[2]-pos_inicial[2]}
end

function leerOrientacionRobot()
    -- Devuelve la orientación del robot en grados, con respecto a la orientación inicial
    -- Usa la orientación del eje X del robot (rojo) con respecto al eje vertical (azul)
    -- El resultado se devuelve en Grados, siempre entre 0 y 360 grados (0: la orientación inicial)
    local o = sim.getObjectOrientation(Robot, -1)
    return ((o[3] - ort_inicial[3])*180/math.pi)%360 
end

function calcDistancia(a, b)
    -- Calcula la distancia lineal entre dos posiciones (X,Y)
    local e = {b[1]-a[1], b[2]-a[2]}
    return math.sqrt(e[1]*e[1]+e[2]*e[2])
end

function crearPosicion(x,y)
    return {x,y}
end

function pulgadasMetros(pulg)
    -- Convierte de pulgadas a metros
    return pulg*0.0254
end

function metroPulgadas(mtr)
    -- Convierte de metros a pulgadas
    return mtr/0.0254
end
-- See the user manual or the available code snippets for additional callback functions and details
function strPosicion(pos)
    return string.format("(%.4f, %.4f)", pos[1], pos[2])
end
