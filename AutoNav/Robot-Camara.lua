function inicializacion()
    --out=sim.auxiliaryConsoleOpen("Debug", 15,1)
    camara = sim.getObjectHandle(sim.handle_self)
    
    local p = 0.6 -- 0.92 color original
    CYAN = {0, p, p}
    MAGENTA = {p, 0 , p}
    NARANJA = {p, p*p, 0}
    ROJO = {p, 0, 0}

    AMARILLO = {0.92, 1, 0.06}


    ROJO_TOLERANCE = {0.2,0.1,0.1}
    COL_TOLERANCE = {0.5,0.5,0.15}


end

function sysCall_vision(inData)

    local pack = leerCamaraBlobs(inData.handle)

    local outData={}
    outData.trigger=true

    outData.packedPackets={pack}
    return outData

end

function ejecucion()
    --leerCamara(camara)
    --sim.auxiliaryConsolePrint(out, "\n Hello")
    --printBlobs(detect)
    --print(leerCentroBlob( detect,  findMaxBlob(detect) ))
    
end

function leerCamaraBlobs(sensor)
    simVision.sensorImgToWorkImg(sensor)
    --simVision.intensityScaleOnWorkImg(camara,0.1, 1,false)   
    --table_3 nominalColor,table_3 colorTolerance,boolean rgbSpace,boolean keepColor,boolean toBuffer1
    --simVision.normalizeWorkImg(sensor)
    -- simVision.edgeDetectionOnWorkImg(sensor, 0.8)
    simVision.selectiveColorOnWorkImg(sensor, AMARILLO, COL_TOLERANCE, true, true, false)
    --number trigger,string packedPacket=
    --simVision.blobDetectionOnWorkImg(
    --    number handle,number threshold,number minBlobSize,bool diffColor,table_3 overlayColor={1.0,0.0,1.0}))
    local trig, pack = simVision.blobDetectionOnWorkImg(sensor, 0.1, .005, false, {1.0,0.0,1.0})
    -- local detect =  sim.unpackFloatTable(pack)
    simVision.workImgToSensorImg(sensor)
    -- Buscar un color para hacerle blob detection
    --local pack = {}
    return pack
end