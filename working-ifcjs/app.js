import {IfcAPI, IFCCARTESIANPOINT} from "web-ifc/web-ifc-api";
const ifcFileLocation = "mccormick-0416-research.ifc";
let modelID = 0;

function getIfcFile(url) {
    return new Promise((resolve,reject) => {
        var oReq = new XMLHttpRequest();
        oReq.responseType = "arraybuffer";
        oReq.addEventListener("load", () => {
            resolve(new Uint8Array(oReq.response));
        });
        oReq.open("GET", url);
        oReq.send();
    });
}

const ifcapi = new IfcAPI();
ifcapi.Init().then(()=>{
    getIfcFile(ifcFileLocation).then((ifcData) => {
        console.log("Reached IFCJS");
        modelID = ifcapi.OpenModel(ifcData);
        let lines = ifcapi.GetLineIDsWithType(modelID,IFCCARTESIANPOINT);
        for(let i = 0; i < lines.size(); i++)
            ;//console.log(ifcapi.GetLine(modelID,lines.get(i)))
    });
});
