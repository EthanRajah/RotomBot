import re
import spacy


def extract_buildings_simple(text):
    """
    Extract building identifiers using regex pattern matching.
    This is a simple approach that works well for consistent patterns like "building X".

    Args:
        text (str): Input text containing building mentions

    Returns:
        list: List of extracted building identifiers
    """
    # Pattern matches "building" followed by letters, numbers, and basic punctuation
    pattern = r'building\s+([A-Za-z0-9][A-Za-z0-9\s\-\.,]*)'

    # Find all matches
    matches = re.finditer(pattern, text, re.IGNORECASE)

    # Extract just the identifier part (after "building")
    buildings = [match.group(1).strip() for match in matches]

    return buildings


def extract_buildings_nlp(text):
    """
    Extract building identifiers using spaCy NLP for more complex scenarios.
    This can handle more varied references to buildings.

    Args:
        text (str): Input text containing building mentions

    Returns:
        list: List of extracted building identifiers
    """
    try:
        # Load spaCy model - you'll need to install it with: python -m spacy download en_core_web_sm
        nlp = spacy.load("en_core_web_sm")
    except OSError:
        print("SpaCy model not found. Please install it with: python -m spacy download en_core_web_sm")
        return extract_buildings_simple(text)  # Fall back to simple method

    # Process the text
    doc = nlp(text)

    buildings = []

    # Look for noun phrases that contain the word "building"
    for chunk in doc.noun_chunks:
        if "building" in chunk.text.lower():
            # Extract just the part after "building"
            match = re.search(r'building\s+([A-Za-z0-9][A-Za-z0-9\s\-\.,]*)', chunk.text, re.IGNORECASE)
            if match:
                buildings.append(match.group(1).strip())

    # If that didn't work well, fall back to regex with the NLP context
    if not buildings:
        # Find mentions of "building" and capture just the identifier
        for token in doc:
            if token.text.lower() == "building":
                # Get the next token to capture the identifier (A, B, C, etc.)
                for next_token in token.rights:
                    buildings.append(next_token.text.strip())
                    break

    # If still no buildings found, use the regex approach
    if not buildings:
        return extract_buildings_simple(text)

    return buildings


def extract_buildings_list(text):
    """
    Extract a list of building identifiers from text that contains a comma-separated list.
    This function handles the specific case of "perform inspection on building A, building B, building C".

    Args:
        text (str): Input text containing a list of buildings

    Returns:
        list: List of extracted building identifiers (just A, B, C, etc.)
    """
    # First, see if we can identify the "perform inspection on" pattern
    match = re.search(r'perform\s+\w+\s+on\s+(.*)', text, re.IGNORECASE)

    if match:
        # Get the part after "perform inspection on"
        building_list_text = match.group(1)

        # Split by comma and "and"
        items = re.split(r',\s*|\s+and\s+', building_list_text)

        # Clean up each item and extract just the identifier
        buildings = []
        for item in items:
            if item.strip():
                # Extract just the part after "building"
                id_match = re.search(r'building\s+([A-Za-z0-9][A-Za-z0-9\s\-\.,]*)', item, re.IGNORECASE)
                if id_match:
                    # Remove any punctuation from id
                    id = re.sub(r'[^\w\s]', '', id_match.group(1))
                    buildings.append(id.strip())

        return buildings

    # If the specific pattern isn't found, try the NLP approach
    return extract_buildings_nlp(text)


if __name__ == "__main__":
    import requests

    input_file = r"ab.txt"
    API_URL = "http://10.42.0.101:5072/api/building/event"

    with open(input_file, 'r') as file:
        lines = file.readlines()

    for line in lines:
        line = line.strip()
        if not line:
            continue

        print(f"\nInput: {line}")
        buildings = extract_buildings_list(line)
        print(f"Extracted building identifiers: {buildings}")

        # Write results to output file
        with open("extracted_buildings.txt", 'a') as output_file:
            output_file.write(f"Input: {line}\n")
            output_file.write(f"Building IDs: {', '.join(buildings)}\n\n")

        # Send each building to your API
        for building_id in buildings:
            payload = {
                "building": building_id,
                "status": "sent",
                "manual_inspection_needed": False
            }

            try:
                resp = requests.post(API_URL, json=payload)
                if resp.status_code == 201:
                    event_id = resp.json().get("event_id")
                    print(f"Sent building {building_id} to API (event_id: {event_id})")
                else:
                    print(f"Failed to send building {building_id}: {resp.text}")
            except Exception as e:
                print(f"Error sending building {building_id} to API: {e}")

    print("\nDone. Results saved to 'extracted_buildings.txt'")
