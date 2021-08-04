import './App.css';
import {useEffect, useState} from "react";
import {Editor} from "./Editor";

export const server = "http://localhost:4567"

function App() {
    const [configs, setConfigs] = useState([])

    async function updateConfigs() {
        let configurations = (await fetch(`${server}/configurations`).then(n => n.text()).then(text => text.split(","))).filter(Boolean)
        setConfigs(configurations)
    }

    useEffect(() => {
        // Update the document title using the browser API
        updateConfigs()
    }, []);

    return (
        <>


            {
                configs && <Editor name={configs[0]} configs={configs} updateHook={updateConfigs}/>
            }
        </>
    );
}

export default App;
