FROM eclipse-temurin:19-jdk


# Install system dependencies
RUN apt update && apt install -y \
    curl \
    unzip \
    ca-certificates \
    libxrender1 \
    libxtst6 \
    libxi6 \
    libxrandr2 \
    libxss1 \
    libxt6 \
    libgl1 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libgtk-3-0 \
    libasound2 \
    wget \
    maven

# Download JavaFX SDK
RUN curl -fL -o javafx.zip https://download2.gluonhq.com/openjfx/19/openjfx-19_linux-x64_bin-sdk.zip && \
    unzip javafx.zip && \
    mv javafx-sdk-19 /opt/javafx && \
    rm javafx.zip
WORKDIR /app

# Copy and build project
COPY pom.xml ./
RUN mkdir -p src/main/java src/main/resources
RUN mvn dependency:go-offline

COPY . .
RUN mvn clean install
COPY Email-EuAll.txt /app/Email-EuAll.txt
# Launch JavaFX app
CMD ["java", "--module-path", "/opt/javafx/lib:/app/target", "--add-modules", "javafx.controls,javafx.fxml", "-jar", "target/graph-algorithms-1.0.0.jar"]
