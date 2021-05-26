function sysCall_init()
    -- do some initialization here

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

    v_0 = 0
    v_max = 50 --cm/s
end

-- una funcion que toma una velocidad
-- y le manda la misma a los otros motores

function sysCall_actuation()
    -- put your actuation code here
    -- avanzar(math.pi/2) --- envia la misma vel a ambos motores
    girar_avanzar(math.pi/2)
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
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

-- Una Funcion que tome una velocidad y se la pase a los motores
--[[
        velocidad(30) --> motorI(30), motorD(30)
--]]

function avanzar(vel)
    mover(vel,vel)
end

function girar_avanzar(vel)
    mover(vel, vel/2)
end


function girarIzq(vel)
    mover(vel, -vel)
end

function girarDer(vel)
    mover(-vel, vel)
end

function mover(velD, velI)

    -- Enviar el comando de velocidad a cada motor
    motorD(velD)
    motorI(velI)
end

-- See the user manual or the available code snippets for additional callback functions and details
