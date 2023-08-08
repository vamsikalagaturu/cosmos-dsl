import json
import pyld.jsonld

jsonld_data = """
{
    "@context": {
        "pidController": "http://example.com/pid_controller#",
        "PIDController": "pidController:PIDController",
        "p-gain": {
            "@id": "pidController:p-gain",
            "@type": "xsd:float"
        },
        "i-gain": {
            "@id": "pidController:i-gain",
            "@type": "xsd:float"
        },
        "d-gain": {
            "@id": "pidController:d-gain",
            "@type": "xsd:float"
        },
        "time-step": {
            "@id": "pidController:time-step",
            "@type": "xsd:float"
        },
        "constraint": {
            "@id": "pidController:constraint",
            "@type": "@id"
        }
    }
}
"""

jsonld_doc = json.loads(jsonld_data)
expanded = pyld.jsonld.expand(jsonld_doc)

print(expanded)
