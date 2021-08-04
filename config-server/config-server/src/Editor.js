import {useEffect, useState} from "react";
import {server} from "./App";
import TOML from "@iarna/toml";
import {JsonTree} from "react-editable-json-tree"

export function Editor(props) {
    const [name, setName] = useState(props.name);
    const [data, setData] = useState("");

    useEffect(() => {
        // Update the document title using the browser API
        (async () => {
            let data = await fetch(`${server}/configurations/${name}`).then(n => n.text())
            setData(data)
        })()
    }, [name]);

    async function onSave() {
        await fetch(`${server}/configurations/${name}/save`, {method: "POST", body: data})
        await fetch(`${server}/configurations/${name}/activate`, {method: "POST"})
    }

    function updateDataJSON(field, value) {
        let jsonData = TOML.parse(data)
        jsonData[field] = value
        setData(TOML.stringify(jsonData))
    }

    return <>
        {
            props.configs.map(n =>
                <button key={n} onClick={() => {
                    console.log(n)
                    setName(n)
                }}>
                    {n}
                </button>
            )
        }
        <div style={{display: "flex",     height: "100%",
        padding: 0,
        margin: 0,
        alignItems: "center",
        justifyContent: "center",
        flexDirection: "column"
    }}>
    Filename: <input value={name} onChange={event => {
        setName(event.target.value)
    }}/>
        Configuration: <textarea value={data} onChange={event => setData(event.target.value)}/>
        <button onClick={onSave}>Save and Activate</button>


    </div>
        </>
}