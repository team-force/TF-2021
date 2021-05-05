function inicializacion()
    --out=sim.auxiliaryConsoleOpen("Debug", 15,1)
    camara = sim.getObjectHandle("Camara_Sensor")
    
    local p = 0.6 
    CYAN = {0, p, p}
    MAGENTA = {p, 0 , p}
    NARANJA = {p, p*p, 0}
    ROJO = {p, 0, 0}

    ROJO_TOLERANCE = {0.2,0.1,0.1}
    COL_TOLERANCE = {0.1,0.1,0.1}


end


function sysCall_vision(inData)

    sensor = inData.handle
    simVision.sensorImgToWorkImg(sensor)
    
    simVision.selectiveColorOnWorkImg(sensor, CYAN, COL_TOLERANCE, true, true, false)
    -- Uso: 
    -- simVision.blobDetectionOnWorkImg(
    --    number handle,number threshold,number minBlobSize,bool diffColor,table_3 overlayColor={1.0,0.0,1.0}))
    
    local trig, pack = simVision.blobDetectionOnWorkImg(sensor, 0.1, .1, false, {1.0,0.0,1.0})
    
    simVision.workImgToSensorImg(sensor)
    
    -- Devolver cuando se use sim.readVisionSensor
    local outData={}
    outData.trigger=true

    outData.packedPackets={pack}
    return outData


end