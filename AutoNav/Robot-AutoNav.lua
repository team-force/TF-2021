function sysCall_init()
-- do some initialization here
   
    Robot = sim.getObjectHandle(sim.handle_self)
    Field = sim.getObjectHandle("Field")

    -- Sensores: Camara
    camara = sim.getObjectHandle("Camara_Sensor")

    -- Sensores: Sonares
    SFrente_D = sim.getObjectHandle("SFrente_Der") 
    SFrente_I = sim.getObjectHandle("SFrente_Izq") 
    SLado_D = sim.getObjectHandle("SLado_Der") 
    SLado_I = sim.getObjectHandle("SLado_Izq") 

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

    caso = 1
    r1, dist_LI = 0, 100
    r2, dist_FI = 0, 100
    r3, dist_FD = 0, 100
    r4, dist_LD = 0, 100

    LI_pos = {0.30, 0.375}
    LI_h = 0.48023 -- Distancia entre centro del robot y Sensor_LI

end

-- una funcion que toma una velocidad 
-- y le manda la misma a los otros motores

function sysCall_actuation()
    -- Codigo de actuacion (acción) del robot
    
    pos_actual = leerPosicionRobot()
    
    --[[
        1. Avanzar hasta encontrar a la izq.
            ¿Cómo entro?
                Si estamos iniciando (Caso 1)
            ¿Cómo salgo?
                Si estoy en Caso 1 y detecto del LI

        2. Avanzar hasta dejar de detectar en la izq.
            ¿Cómo entro?
                Y estoy en Caso 1 Cuando detecto del LI
            ¿Cómo salgo?
                Cuando dejo de detectar en LI

        3. Girar en arco hasta detectar
            ¿Cómo entro?
                Cuando dejo de detectar en LI
            ¿Cómo salgo?
                Cuando detecto en LI

        4. Mantener el arco hasta detectar X distancia en el sens de la derecha.
            ¿Cómo entro?
            *Cuando detecto en LI 
            ¿Cómo salgo?
                Cuando LD mide <= X distancia.
    --]]
    
    --[[
        Opción 1: avanzar hasta que LD deje de ver el objeto
    ]]

    if caso == 1 then
        if r1 == 0 then
            avanzar(6*v_0) 
        else -- detectamos algo en LI
            caso = 2
        end 

    elseif caso == 2 then
        if r1 ~= 0 then
            avanzar(6*v_0) 
            -- Gire para mantener una dista X del cono
            ultima_LI = dist_LI
            print(dist_LI)
            --caso = 3
        else -- detectamos algo en LI
            caso = 3
        end 
    elseif caso == 3 then
        if dist_LD > pulgadasMetros(30) then
            radio_c3 = radio_objetivo_cono()
            arco(v_0, radio_c3)
            print(radio_c3)
        else
            caso = 4
        end
        print(dist_LD)
    elseif caso == 4 then
        if r4 == 1 then
            avanzar(3*v_0)
        else
            caso = 5
        end
    elseif caso == 5 then
        if r4 == 0 then
            arco(v_0, -radio_c3)
        else
            caso = 6
        end
    elseif caso == 6 then
        if r4 == 1 then
            arco(v_0, -0.9*radio_c3)
        else
            caso = 7
        end
    elseif caso == 7 then
        avanzar(0)
        print("Stop")
        caso = 25
    end
    
    
end

function radio_objetivo_cono()
    --[[ Calcula el radio de un arco que une al robot
         con un punto paralelo al cono que detecta 
         sensor del LadoI (por ahora forzado)
    --]]

    -- Calcular Radio de un arco que una los dos puntos
    -- Calcular la Curvatura (Y=1/r)
    
    -- Para extender la distancia frontal del punto
    local dist_cono_punto = pulgadasMetros(30)
    local dX = (LI_h + ultima_LI)*0.707
    local l = dX
    local L = l + dist_cono_punto
    -- Para extender la distancia lateral del punto
    dX = dX + pulgadasMetros(30)
    local beta = math.atan2(L, dX)
    local D = L/math.sin(beta)
    local R = D^2/(2*dX)
    return R    

end

function sysCall_sensing()
    -- put your sensing code here
    
    r1, dist_LI = sim.readProximitySensor(SLado_I)
    r2, dist_FI = sim.readProximitySensor(SFrente_I)
    r3, dist_FD = sim.readProximitySensor(SFrente_D)
    r4, dist_LD = sim.readProximitySensor(SLado_D)

    if r1 == 0 then
        dist_LI = 100
    end
    if r2 == 0 then
        dist_FI = 100
    end
    if r3 == 0 then
        dist_FD = 100
    end
    if r4 == 0 then
        dist_LD = 100
    end

    
    --print({dist_LI, dist_FI, dist_FD, dist_LD})

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
