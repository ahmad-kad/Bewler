============
Contributing
============

Guidelines for contributing to the URC 2026 project.

Development Workflow
====================

1. **Fork the repository**
2. **Create a feature branch**: ``git checkout -b feature/your-feature``
3. **Make changes** following our coding standards
4. **Write tests** for new functionality
5. **Update documentation** as needed
6. **Submit a pull request**

Coding Standards
================

**Python Code:**

- Follow PEP 8 style guidelines
- Use type hints for function parameters and return values
- Write comprehensive docstrings in Google format
- Maximum line length: 88 characters

**JavaScript/React Code:**

- Use ESLint configuration provided
- Follow React best practices
- Use TypeScript for new components where possible
- Write JSDoc comments for public APIs

**ROS2 Code:**

- Follow ROS2 naming conventions
- Document all messages, services, and actions
- Include proper error handling
- Write unit tests for nodes

Documentation Standards
=======================

- **Code comments**: Explain why, not just what
- **API documentation**: Complete parameter and return descriptions
- **User guides**: Step-by-step instructions with screenshots
- **Architecture docs**: High-level design decisions

Testing Requirements
====================

- **Unit tests**: 90%+ code coverage for new code
- **Integration tests**: End-to-end functionality verification
- **Documentation tests**: Doctest for code examples
- **Performance tests**: Benchmark critical paths

Review Process
==============

All changes require:

1. **Automated checks**: CI must pass
2. **Code review**: At least one maintainer approval
3. **Testing**: All tests must pass
4. **Documentation**: Updated as needed

Getting Help
============

- **Issues**: Use GitHub issues for bugs and features
- **Discussions**: Use GitHub discussions for questions
- **Documentation**: Check docs first, then ask
- **Code reviews**: Be specific about concerns and suggestions
