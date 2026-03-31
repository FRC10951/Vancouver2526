Add-Type -AssemblyName System.Speech
$speak = New-Object System.Speech.Synthesis.SpeechSynthesizer
$speak.Volume = 100
$message = "Check the Vancouver 25 26 groupchat on Google Chat"
$speak.Speak($message)
$speak.Speak($message)
