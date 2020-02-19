require "math"

function extsysCall_init()
    h=sim.getObjectAssociatedWithScript(sim.handle_self)
    m=sim.getShapeMassAndInertia(h)
    simRemoteApi.start(8080)

    moveSub=simROS.subscribe('/sim/move', 'std_msgs/Float32MultiArray', 'move_callback')
    simROS.subscriberTreatUInt8ArrayAsString(moveSub)

    HEAD_FLAG = -525600
    rospubs = {}
    last = nil
    seq = 0;    
end

function extsysCall_actuation() 

end

function move_callback(msg)
    last = msg.data
    print(last)
end

function mysplit (inputstr, sep)
    if sep == nil then
            sep = "%s"
    end
    local t={}
    for str in string.gmatch(inputstr, "([^"..sep.."]+)") do
            table.insert(t, str)
    end
    return t
end

function get_ros_data(inInts, inFloats, inString, inBuffer)
	if #inString >= 2 then
		i = 1
		head = {}
	    head["seq"] = seq
	    seq = seq + 1
	    head["stamp"] = simROS.getTime()
	    head["frame_id"] = "global"
		while i <= #inString do
			if string.sub(inString[i],1,1)== '/' then
				if rospubs == {} then
					rospubs[#rospubs+1] = simROS.advertise(inStrings[i], inStrings[i+1])
		    		simROS.publisherTreatUInt8ArrayAsString(rospubs[#rospubs])
		    	end
	    		i = i + 2
			end
			temp = rospubs[#rospubs]
			splitstr = mysplit(inString[i], '.')
			for k = 1,#splitstr do
				path = splitstr[k]
				if temp[path] == nil then
					temp[path] = {}
				end
				if path ~= splitstr[#splitstr] then
					temp = temp[path]
				end
				lastpath = path
			end
			temp[lastpath] = inFloats[i]
			if temp[lastpath] == HEAD_FLAG then
				temp[lastpath] = head
			end
			i = i + 1
		end
	end
	if last == nil then
        return {}, {0,0,0,0,0,0,0,0}, {}, ''
    end
    return {}, last, {}, ''
end





































