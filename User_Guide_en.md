# Myoung-ja User Guide

<b>Companion robot Myoung-ja for elderly individuals living alone</b>

## Table of Contents
[1. Robot](#1-robot)  
[2. Battery](#2-battery)   
[3. Patting](#3-patting)   
[4. Emotion Analysis](#4-emotion-analysis)   
[5. Medication Reminder](#5-medication-reminder)
[6. Motion Control](#6-motion-control)
[7. Gas Sensor (Hazard Detection-1)](#7-gas-sensor-hazard-detection-1)   
[8. Fall Detection (Hazard Detection-2)](#8-fall-detection-hazard-detection-2)   
[9. Let's Take a Walk](#9-lets-take-a-walk)  
[10. Follow Me](#10-follow-me)  
[11. Shutdown](#11-shutdown)

## Main Text
### 1. Robot
Myoung-ja is always in ***standby mode*** and activates when the user calls out ***Hi*** Additionally, while active,    
Myoung-ja will randomly ***initiate a conversation with the user***

### 2. Battery
The user can inquire about Myoung-ja's <b>remaining battery level and usage time</b> by saying 'battery.'

Example response:
```shell
"The battery level is 30 percent. There are 2 hours and 30 minutes of usage time remaining."
```

### 3. Patting
When the user pats Myoung-ja's head, Myoung-ja blinks its eyes.

### 4. Emotion Analysis
Myoung-ja analyzes the user's emotions through speech with ChatGPT and changes its facial expression accordingly.

Example: Utterance - Emotion
```
"My parents are unwell" - (Sad)   
"I was deceived by a friend" - (Angry)
"I won the lottery" - (Moving)
"Today it's raining" - (Normal)

```

### 5. Medication Reminder
Myoung-ja informs the user of medication times based on stored user data.

Condition / Medication Time - Utterance
```shell
(Hypertension patient / 12 PM) - "It's time to take your hypertension medication!"
```

### 6. Motion Control
The user can control some of Myoung-ja's movements through voice commands.

Command - Motion

```shell 
Hug me - Myoung-ja extends its arms to hug the user.
Right hand - Myoung-ja raises its right hand.
Left hand - Myoung-ja raises its left hand.
```

### 7. Gas Sensor (Hazard Detection-1)
Using a gas sensor (MQ-7), Myoung-ja measures carbon monoxide levels to detect fires.    
In the event of a fire, Myoung-ja sends an emergency report via text, including the situation and GPS-determined location.

### 8. Fall Detection (Hazard Detection-2)
Myoung-ja detects user falls and inquires about their well-being.

In the event of a fall:
```shell
"Are you okay?"
```

### 9. Let's Take a Walk
By saying 'Let's take a walk,' the user prompts Myoung-ja to raise its arm to a position where the user can hold Myja's hand.    
This allows the user to take a walk while holding Myoung-ja's hand.

### 10. Follow Me
Saying 'Follow me' prompts Myoung-ja to identify the user and follow them.

### 11. Shutdown
Saying 'Shutdown' prompts Myoung-ja to end its functions.
