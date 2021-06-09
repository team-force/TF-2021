function GSearch_init()
-- do some initialization here
    SensorAbajo = sim.getObjectHandle(sim.handle_self);
    pelotaAttach = sim.getObjectHandle('Attach_Pelota0');
    handDummie = sim.getObjectHandle('BolaSuperior');
end


function pelota_en_zona()

    local result,distance,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.readProximitySensor(SensorAbajo)

    -- Eventualmente, deberiamo revisar si detectamos una pelota
    if detectedObjectHandle then
        -- local x = sim.getObjectPosition(detectedObjectHandle,SensorAbajo)
        -- print(x)
        local x = {0,0,0}
        -- print(x)
        -- Linea: ubica un objeto (detectObjectHandle), en la posicion X, con respecto a handDummie
        sim.setObjectParent(detectedObjectHandle, pelotaAttach, true)
        sim.setObjectPosition(detectedObjectHandle, pelotaAttach, x)

    end

end
