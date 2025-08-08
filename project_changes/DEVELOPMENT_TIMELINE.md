# SAR Drone Development Timeline (Aug 2-15)

## Week 1 (Aug 2-8): Core Functionality

### Days 1-2: Search Patterns Implementation
- Grid search pattern
- Spiral search pattern
- Expanding square pattern
- Integration with LLM commands ("search the area", "perform grid search")

### Days 3-4: Vision-Guided Navigation
- Target tracking from detections
- Navigate to detected persons
- Maintain visual lock during approach
- Integration with scene descriptions

### Days 5-6: Basic Obstacle Avoidance
- Simple reactive avoidance
- Maintain mission objectives while avoiding
- Safety distance parameters

### Day 7: YOLO Model Fine-tuning
- Download Kaggle SAR dataset
- Configure training pipeline
- Start training on RTX 5090

## Week 2 (Aug 9-15): Integration & Demo

### Days 8-9: Model Integration & Testing
- Integrate fine-tuned YOLO model
- Test victim detection accuracy
- Optimize confidence thresholds

### Days 10-11: Mission Scenarios
- Lost hiker search scenario
- Multi-victim disaster scenario
- Urban search scenario
- Night search simulation

### Days 12-13: System Integration
- Full pipeline testing
- Performance optimization
- Bug fixes and refinements

### Day 14: Demo Preparation
- Record demo videos
- Prepare presentation materials
- System documentation

### Day 15: Final Review
- Last-minute fixes
- Submission preparation

## Priority Features for Demo

### Must Have (by Aug 10)
1. Working search patterns
2. Person detection with SAR-tuned model
3. Basic obstacle avoidance
4. LLM command integration

### Nice to Have (if time permits)
1. Advanced mission planning
2. Multi-waypoint navigation
3. Detailed logging system
4. Heat map generation

## Daily Development Goals

- **2-3 hours**: Core feature implementation
- **1 hour**: Testing and debugging
- **30 min**: Documentation updates
- **30 min**: Demo scenario development

## Key Milestones

- **Aug 5**: First search pattern working
- **Aug 8**: YOLO model trained
- **Aug 10**: All core features integrated
- **Aug 13**: Demo scenarios complete
- **Aug 15**: Project submission